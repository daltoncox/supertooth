/**
 * @file app_device_view.c
 * @brief Live 1 Hz device/connection table for the CLI `-v devices` mode.
 *
 * Polls the core's protocol-agnostic device/connection snapshots (the same
 * source the GUI's DeviceListView uses) and renders a table whose columns and
 * row derivation mirror the GUI exactly: RSSI, Protocol, Type, Identifier,
 * First Seen, Last Seen, Packet Rate. The 1-second packet rate is derived
 * from the delta of each entity's total packet count between polls.
 *
 * On a TTY the table is redrawn in place (cursor-up + clear of only the table
 * region) so it stays put on screen; when stdout is piped the table is
 * appended each second (with a timestamp) so logs capture every snapshot.
 */
#include "app_device_view.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "app_common.h"
#include "device_models.h"
#include "session.h"

/* Capacity for a single poll's merged entity list. */
#define DEV_VIEW_MAX_ENTITIES 2048u
/* Capacity of the (kind,id) -> previous-total packet cache used for rate. */
#define DEV_VIEW_RATE_CACHE   2048u

typedef struct
{
    uint64_t       key;          /* (kind << 56) | (id & 0x00FF...FF) */
    unsigned long  prev_total;
} rate_cache_entry_t;

/* One merged, display-ready entity row. */
typedef struct
{
    char    proto[8];                       /* "BR/EDR" / "LE" */
    char    addr[DEVICE_ADDR_STR_MAX];
    char    device[DEVICE_LABEL_MAX];       /* core label (Central/LT_ADDR N/...) */
    char    addr_type[DEVICE_ADDRTYPE_MAX];
    char    name[DEVICE_NAME_MAX];
    char    manufacturer[DEVICE_MANUF_MAX];
    char    appearance[DEVICE_APPEARANCE_MAX];
    char    flags[DEVICE_FLAGS_MAX];
    char    device_class[DEVICE_COD_MAX];
    uint64_t id;
    int     kind;
    float   rssi_db;
    int     rssi_valid;
    uint64_t first_seen_ms;
    uint64_t last_seen_ms;
    unsigned long total_packets;
    unsigned int  packet_rate;
    uint64_t group_id;      /* BR/EDR piconet linkage: member -> owning
                               connection_id, connection -> own id,
                               0 = standalone / LE (no group) */
    int     lt_slot;        /* -1 = BR/EDR connection, 255 = central,
                               0..7 = peripheral, -2 = other */
    int     is_last_child;  /* 1 when last member of its piconet group */
    int     has_children;   /* 1 when a connection has member rows beneath it */
    char    type[32];                       /* derived Type column */
    char    identifier[96];                 /* derived Identifier column */
} dev_entity_t;

struct app_device_view
{
    session_t  *session;
    pthread_t   thread;
    _Atomic int stop;
    int         is_tty;
    size_t      prev_lines;   /* lines the last table occupied (TTY redraw) */
    rate_cache_entry_t *rate_cache;
    size_t      rate_cache_count;
};

/* ---------------------------------------------------------------------------
 * Formatting helpers (mirror DeviceListModel QML formatting)
 * ---------------------------------------------------------------------------*/

/* Replicates DeviceListModel::typeLabelFor(proto, device, addrType). */
static void compute_type(const char *proto, const char *device,
                         const char *addr_type, char *out, size_t n)
{
    if (strcmp(proto, "LE") == 0)
    {
        if (strcmp(device, "connection") == 0) { snprintf(out, n, "CONN"); return; }
        if (addr_type && addr_type[0])         { snprintf(out, n, "%s", addr_type); return; }
        snprintf(out, n, "ADV_ADDR"); return;
    }
    if (strcmp(proto, "BR/EDR") == 0)
    {
        if (strcmp(device, "connection") == 0)  { snprintf(out, n, "CONN"); return; }
        if (strcmp(device, "INQUIRY") == 0)  { snprintf(out, n, "INQUIRY"); return; }
        snprintf(out, n, "LT_ADDR"); return;
    }
    snprintf(out, n, "--");
}

/* Mirrors DeviceListModel::identifierLabelFor(row): BR/EDR piconets render
 * hardcoded-expanded (connection as plain address parent, members as
 * ├─/└─ children with short labels). INQUIRY / standalone / LE rows keep
 * the old format. */
static int is_piconet_connection(const dev_entity_t *e)
{
    return strcmp(e->proto, "BR/EDR") == 0 &&
           strcmp(e->device, "connection") == 0 && e->lt_slot == -1;
}

static int is_piconet_member(const dev_entity_t *e)
{
    return strcmp(e->proto, "BR/EDR") == 0 &&
           e->group_id != 0u && e->lt_slot >= 0;
}

static int tree_rank(const dev_entity_t *e)
{
    if (is_piconet_connection(e))
        return 0;
    if (e->lt_slot == 255)
        return 1;
    return 2;
}

static void compute_identifier(const char *proto, const char *device,
                               const char *addr, const char *addr_type,
                               const char *name, int lt_slot, uint64_t group_id,
                               int is_last, int has_children,
                               char *out, size_t n)
{
    (void)addr_type;
    (void)has_children;
    if (strcmp(proto, "BR/EDR") == 0)
    {
        if (strcmp(device, "INQUIRY") == 0)
        {
            snprintf(out, n, "%s", addr ? addr : "");
            return;
        }
        if (strcmp(device, "connection") == 0 && lt_slot == -1)
        {
            snprintf(out, n, "%s", addr ? addr : "");
            return;
        }
        if (group_id != 0u && lt_slot >= 0)
        {
            const char *prefix = is_last ? "└─ " : "├─ ";
            if (strcmp(device, "Central") == 0)
                snprintf(out, n, "%sCentral", prefix);
            else
                snprintf(out, n, "%s%s", prefix, device);
            return;
        }
        const char *suffix = "";
        if (strcmp(device, "Central") == 0)
            suffix = "C";
        else if (strncmp(device, "LT_ADDR ", 8) == 0)
            suffix = device + 8;
        else
            suffix = device;
        if (suffix[0] == '\0')
        {
            snprintf(out, n, "%s", addr ? addr : "");
            return;
        }
        snprintf(out, n, "%s-%s", addr ? addr : "", suffix);
        return;
    }
    if (strcmp(device, "connection") == 0)
    {
        snprintf(out, n, "%s", (addr && addr[0]) ? addr : "Connection");
        return;
    }
    if (name && name[0])
    {
        snprintf(out, n, "%s", name);
        return;
    }
    snprintf(out, n, "%s", (addr && addr[0]) ? addr : "Unknown");
}

static void fmt_rssi(int valid, float db, char *out, size_t n)
{
    if (!valid) { snprintf(out, n, "--"); return; }
    snprintf(out, n, "%.1f", (double)db);
}

static void fmt_ts(uint64_t ms, char *out, size_t n)
{
    if (ms == 0u) { snprintf(out, n, "--"); return; }
    time_t sec = (time_t)(ms / 1000u);
    struct tm tmv;
    localtime_r(&sec, &tmv);
    int tenth = (int)((ms % 1000u) / 100u);
    snprintf(out, n, "%02d:%02d:%02d.%d",
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec, tenth);
}

static void fmt_now(char *out, size_t n)
{
    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    snprintf(out, n, "%02d:%02d:%02d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

static int cmp_identifier(const void *a, const void *b)
{
    const dev_entity_t *ea = (const dev_entity_t *)a;
    const dev_entity_t *eb = (const dev_entity_t *)b;
    int t = strcmp(ea->proto, eb->proto);
    if (t != 0)
        return t;
    /* BR/EDR piconets stay contiguous: connection first, then Central,
     * then LT_ADDR numeric. Grouped rows sort before ungrouped BR/EDR
     * singletons so a group is never split. */
    if (strcmp(ea->proto, "BR/EDR") == 0)
    {
        int ga = is_piconet_connection(ea) || is_piconet_member(ea);
        int gb = is_piconet_connection(eb) || is_piconet_member(eb);
        if (ga && gb)
        {
            /* Group by stable piconet linkage, not the address string:
             * member rows force the UAP to known while the connection
             * shows 0x?? until UAP recovery. Piconets order by LAP (addr
             * suffix); the numeric group_id only breaks LAP ties. Parent
             * always first; only piconet-to-piconet order is directional
             * (CLI is ascending-only). Final tie-breaks use raw fields,
             * never the ├─/└─ prefixed identifier (assigned post-sort). */
            if (ea->group_id != eb->group_id)
            {
                const char *as = ea->addr;
                const char *bs = eb->addr;
                size_t al = strlen(as), bl = strlen(bs);
                const char *ak = (al >= 6u) ? as + al - 6u : as;
                const char *bk = (bl >= 6u) ? bs + bl - 6u : bs;
                int lt = strcmp(ak, bk);
                if (lt != 0)
                    return lt;
                return (ea->group_id < eb->group_id) ? -1 : 1;
            }
            int ra = tree_rank(ea);
            int rb = tree_rank(eb);
            if (ra != rb)
                return ra - rb;
            if (ea->lt_slot != eb->lt_slot)
                return ea->lt_slot - eb->lt_slot;
            t = strcmp(ea->addr, eb->addr);
            if (t != 0)
                return t;
            return strcmp(ea->device, eb->device);
        }
        if (ga != gb)
            return ga ? -1 : 1;
    }
    t = strcmp(ea->type, eb->type);
    if (t != 0)
        return t;
    return strcmp(ea->identifier, eb->identifier);
}

/* Mark the last member of each piconet group (└─ vs ├─). Expects rows in
 * display order. */
static void assign_tree_flags(dev_entity_t *e, size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        e[i].is_last_child = 0;
        e[i].has_children = 0;
    }
    for (size_t i = 0; i < n; i++)
    {
        if (!is_piconet_member(&e[i]))
            continue;
        int last = 1;
        for (size_t j = i + 1u; j < n; j++)
        {
            if (is_piconet_member(&e[j]) && e[j].group_id == e[i].group_id)
            {
                last = 0;
                break;
            }
        }
        e[i].is_last_child = last;
    }
    for (size_t i = 0; i < n; i++)
    {
        if (!is_piconet_connection(&e[i]))
            continue;
        for (size_t j = 0; j < n; j++)
        {
            if (is_piconet_member(&e[j]) &&
                (e[j].group_id == e[i].group_id || e[j].group_id == e[i].id))
            {
                e[i].has_children = 1;
                break;
            }
        }
    }
}

/* 1-second packet rate = delta of total_packets since the previous poll. */
static unsigned int compute_rate(app_device_view_t *v, int kind, uint64_t id,
                                 unsigned long total)
{
    uint64_t key = ((uint64_t)(unsigned int)kind << 56)
                 | (id & 0x00FFFFFFFFFFFFFFu);
    for (size_t i = 0; i < v->rate_cache_count; i++)
    {
        if (v->rate_cache[i].key == key)
        {
            unsigned long prev = v->rate_cache[i].prev_total;
            v->rate_cache[i].prev_total = total;
            if (total >= prev)
                return (unsigned int)(total - prev);
            return 0u;
        }
    }
    if (v->rate_cache_count < DEV_VIEW_RATE_CACHE)
    {
        v->rate_cache[v->rate_cache_count].key = key;
        v->rate_cache[v->rate_cache_count].prev_total = total;
        v->rate_cache_count++;
    }
    return 0u;
}

/* ---------------------------------------------------------------------------
 * Poll + render
 * ---------------------------------------------------------------------------*/

/* Pull all four entity lists from the core and merge into @p out (cap
 * DEV_VIEW_MAX_ENTITIES). Returns the merged count. */
static size_t collect(app_device_view_t *v, dev_entity_t *out)
{
    const size_t cap = DEV_VIEW_MAX_ENTITIES;
    bredr_device_snapshot_t     *bd = calloc(cap, sizeof(*bd));
    bredr_connection_snapshot_t *bp = calloc(cap, sizeof(*bp));
    ble_device_snapshot_t       *ld = calloc(cap, sizeof(*ld));
    ble_connection_snapshot_t   *lp = calloc(cap, sizeof(*lp));
    if (!bd || !bp || !ld || !lp)
    {
        free(bd); free(bp); free(ld); free(lp);
        return 0u;
    }

    size_t nb = session_get_bredr_devices(v->session, bd, cap);
    size_t np = session_get_bredr_connections(v->session, bp, cap);
    size_t nl = session_get_ble_devices(v->session, ld, cap);
    size_t nq = session_get_ble_connections(v->session, lp, cap);

    size_t n = 0u;

    for (size_t i = 0; i < nb && n < cap; i++)
    {
        dev_entity_t *e = &out[n++];
        memset(e, 0, sizeof(*e));
        e->id = bd[i].id; e->kind = (int)bd[i].kind;
        snprintf(e->proto, sizeof(e->proto), "BR/EDR");
        snprintf(e->addr, sizeof(e->addr), "%s", bd[i].addr_str);
        snprintf(e->device, sizeof(e->device), "%s", bd[i].label);
        e->group_id = bd[i].connection_id;
        e->lt_slot = (bd[i].connection_id == 0u) ? -2 : (int)bd[i].lt_addr;
        e->is_last_child = 0;
        e->has_children = 0;
        e->rssi_valid = bd[i].rssi_valid; e->rssi_db = bd[i].rssi_db;
        e->first_seen_ms = bd[i].first_seen_ms;
        e->last_seen_ms = bd[i].last_seen_ms;
        e->total_packets = bd[i].total_packets;
    }
    for (size_t i = 0; i < np && n < cap; i++)
    {
        dev_entity_t *e = &out[n++];
        memset(e, 0, sizeof(*e));
        e->id = bp[i].id; e->kind = (int)bp[i].kind;
        snprintf(e->proto, sizeof(e->proto), "BR/EDR");
        snprintf(e->addr, sizeof(e->addr), "%s", bp[i].addr_str);
        snprintf(e->device, sizeof(e->device), "%s", bp[i].label);
        e->group_id = bp[i].id;
        e->lt_slot = -1;
        e->is_last_child = 0;
        e->has_children = 0;
        e->rssi_valid = bp[i].rssi_valid; e->rssi_db = bp[i].rssi_db;
        e->first_seen_ms = bp[i].first_seen_ms;
        e->last_seen_ms = bp[i].last_seen_ms;
        e->total_packets = bp[i].total_packets;
    }
    for (size_t i = 0; i < nl && n < cap; i++)
    {
        dev_entity_t *e = &out[n++];
        memset(e, 0, sizeof(*e));
        e->id = ld[i].id; e->kind = (int)ld[i].kind;
        snprintf(e->proto, sizeof(e->proto), "LE");
        snprintf(e->addr, sizeof(e->addr), "%s", ld[i].addr_str);
        snprintf(e->device, sizeof(e->device), "%s", ld[i].label);
        snprintf(e->addr_type, sizeof(e->addr_type), "%s", ld[i].addr_type);
        snprintf(e->name, sizeof(e->name), "%s", ld[i].name);
        snprintf(e->manufacturer, sizeof(e->manufacturer), "%s", ld[i].manufacturer);
        snprintf(e->appearance, sizeof(e->appearance), "%s", ld[i].appearance);
        snprintf(e->flags, sizeof(e->flags), "%s", ld[i].flags);
        snprintf(e->device_class, sizeof(e->device_class), "%s", ld[i].device_class);
        e->rssi_valid = ld[i].rssi_valid; e->rssi_db = ld[i].rssi_db;
        e->first_seen_ms = ld[i].first_seen_ms;
        e->last_seen_ms = ld[i].last_seen_ms;
        e->total_packets = ld[i].total_packets;
    }
    for (size_t i = 0; i < nq && n < cap; i++)
    {
        dev_entity_t *e = &out[n++];
        memset(e, 0, sizeof(*e));
        e->id = lp[i].id; e->kind = (int)lp[i].kind;
        snprintf(e->proto, sizeof(e->proto), "LE");
        snprintf(e->addr, sizeof(e->addr), "%s", lp[i].addr_str);
        snprintf(e->device, sizeof(e->device), "%s", lp[i].label);
        e->id = lp[i].id;
        e->rssi_valid = lp[i].rssi_valid; e->rssi_db = lp[i].rssi_db;
        e->first_seen_ms = lp[i].first_seen_ms;
        e->last_seen_ms = lp[i].last_seen_ms;
        e->total_packets = lp[i].total_packets;
    }

    for (size_t i = 0; i < n; i++)
    {
        /* LE rows carry no piconet linkage; mark explicitly (memset left
         * lt_slot 0, which would collide with peripheral LT_ADDR 0). */
        if (strcmp(out[i].proto, "LE") == 0)
        {
            out[i].group_id = 0u;
            out[i].lt_slot = -2;
            out[i].is_last_child = 0;
            out[i].has_children = 0;
        }
        compute_type(out[i].proto, out[i].device, out[i].addr_type,
                     out[i].type, sizeof(out[i].type));
        /* Provisional prefixes; the post-sort pass in dev_view_tick
         * corrects member tails (└─ vs ├─). The comparator ignores
         * prefixes for grouped rows, so order is unaffected. */
        compute_identifier(out[i].proto, out[i].device, out[i].addr,
                           out[i].addr_type, out[i].name,
                           out[i].lt_slot, out[i].group_id, 0, 0,
                           out[i].identifier, sizeof(out[i].identifier));
    }

    free(bd); free(bp); free(ld); free(lp);
    return n;
}

/* Re-derive tree identifiers after sorting: members get ├─/└─ tails. */
static void relabel_tree_prefixes(dev_entity_t *e, size_t n)
{
    assign_tree_flags(e, n);
    for (size_t i = 0; i < n; i++)
    {
        if (!is_piconet_member(&e[i]) && !is_piconet_connection(&e[i]))
            continue;
        compute_identifier(e[i].proto, e[i].device, e[i].addr,
                           e[i].addr_type, e[i].name,
                           e[i].lt_slot, e[i].group_id,
                           e[i].is_last_child, e[i].has_children,
                           e[i].identifier, sizeof(e[i].identifier));
    }
}

/* Display-cell width of a UTF-8 string: continuation bytes (10xxxxxx) occupy
 * no terminal cells, so subtract them from the byte length. Covers the tree
 * glyphs (├ U+251C, └ U+2514, ─ U+2500, all 3-byte). */
static size_t disp_width(const char *s)
{
    size_t cells = 0u;
    for (const unsigned char *p = (const unsigned char *)s; *p; p++)
        if ((*p & 0xC0u) != 0x80u)
            cells++;
    return cells;
}

/* Copy @p src into @p dst padded with spaces to @p width display cells. */
static void pad_cell(const char *src, char *dst, size_t dstn, size_t width)
{
    size_t w = disp_width(src);
    snprintf(dst, dstn, "%s", src ? src : "");
    if (w >= width)
        return;
    size_t have = strlen(dst);
    size_t pad = width - w;
    if (have + pad + 1u > dstn)
        pad = dstn - have - 1u;
    memset(dst + have, ' ', pad);
    dst[have + pad] = '\0';
}

#define DEV_VIEW_IDENT_WIDTH 26u

/* Print the table; returns the number of lines emitted (for TTY redraw). */
static size_t print_table(app_device_view_t *v, dev_entity_t *e, size_t n)
{
    (void)v;
    size_t lines = 0u;

    printf("%-8s %-7s %-13s %-26s %-12s %-12s %-8s %-9s\n",
           "RSSI", "Proto", "Type", "Identifier", "First", "Last", "Pkts", "Pkts/s");
    lines++;
    printf("-------- ------- ------------- -------------------------- ------------ ------------ -------- ---------\n");
    lines++;

    for (size_t i = 0; i < n; i++)
    {
        char rssi[16], first[16], last[16], idisp[40];
        fmt_rssi(e[i].rssi_valid, e[i].rssi_db, rssi, sizeof(rssi));
        fmt_ts(e[i].first_seen_ms, first, sizeof(first));
        fmt_ts(e[i].last_seen_ms, last, sizeof(last));
        pad_cell(e[i].identifier, idisp, sizeof(idisp), DEV_VIEW_IDENT_WIDTH);
        printf("%-8s %-7s %-13s %s %-12s %-12s %-8lu %-9u\n",
               rssi, e[i].proto, e[i].type, idisp, first, last,
               e[i].total_packets, e[i].packet_rate);
        lines++;
        /* Extra detail line for LE advertisers carrying appearance /
         * flags / class-of-device / manufacturer info. */
        if (strcmp(e[i].proto, "LE") == 0 &&
            (e[i].appearance[0] || e[i].flags[0] ||
             e[i].device_class[0] || e[i].manufacturer[0]))
        {
            char extra[256];
            snprintf(extra, sizeof(extra), "         appearance=%s flags=%s class=%s manuf=%s",
                     e[i].appearance[0] ? e[i].appearance : "--",
                     e[i].flags[0] ? e[i].flags : "--",
                     e[i].device_class[0] ? e[i].device_class : "--",
                     e[i].manufacturer[0] ? e[i].manufacturer : "--");
            printf("%s\n", extra);
            lines++;
        }
    }
    if (n == 0u)
    {
        printf("(no devices — waiting for traffic)\n");
        lines++;
    }
    return lines;
}

static void dev_view_tick(app_device_view_t *v)
{
    dev_entity_t *ents = calloc(DEV_VIEW_MAX_ENTITIES, sizeof(*ents));
    if (!ents)
        return;

    size_t n = collect(v, ents);
    qsort(ents, n, sizeof(*ents), cmp_identifier);
    relabel_tree_prefixes(ents, n);
    for (size_t i = 0; i < n; i++)
        ents[i].packet_rate =
            compute_rate(v, ents[i].kind, ents[i].id, ents[i].total_packets);

    app_output_lock();
    if (v->is_tty)
    {
        if (v->prev_lines > 0u)
        {
            printf("\033[%zuA", v->prev_lines);
            for (size_t i = 0; i < v->prev_lines; i++)
                printf("\033[K\n");
            printf("\033[%zuA", v->prev_lines);
        }
        v->prev_lines = print_table(v, ents, n);
    }
    else
    {
        char ts[32];
        fmt_now(ts, sizeof(ts));
        printf("\n=== %s ===\n", ts);
        print_table(v, ents, n);
    }
    fflush(stdout);
    app_output_unlock();

    free(ents);
}

static void *dev_view_thread(void *arg)
{
    app_device_view_t *v = (app_device_view_t *)arg;

    app_output_lock();
    if (v->is_tty)
        printf("Device list (updates every 1s) — Ctrl+C to stop.\n");
    app_output_unlock();

    while (!atomic_load(&v->stop))
    {
        dev_view_tick(v);
        /* Sleep ~1s in 100ms slices so a stop is honored promptly. */
        for (int i = 0; i < 10 && !atomic_load(&v->stop); i++)
            usleep(100000);
    }
    return NULL;
}

app_device_view_t *app_device_view_start(session_t *session)
{
    if (!session)
        return NULL;

    app_device_view_t *v = calloc(1, sizeof(*v));
    if (!v)
        return NULL;

    v->session = session;
    atomic_store(&v->stop, 0);
    v->is_tty = isatty(fileno(stdout));
    v->prev_lines = 0u;
    v->rate_cache = calloc(DEV_VIEW_RATE_CACHE, sizeof(rate_cache_entry_t));
    if (!v->rate_cache)
    {
        free(v);
        return NULL;
    }
    v->rate_cache_count = 0u;

    if (pthread_create(&v->thread, NULL, dev_view_thread, v) != 0)
    {
        free(v->rate_cache);
        free(v);
        return NULL;
    }
    return v;
}

void app_device_view_stop(app_device_view_t *view)
{
    if (!view)
        return;

    atomic_store(&view->stop, 1);
    pthread_join(view->thread, NULL);

    if (view->is_tty && view->prev_lines > 0u)
    {
        /* Drop the cursor below the final table so the session summary
         * that follows prints on its own line. */
        printf("\033[%zuB\n", view->prev_lines);
        fflush(stdout);
    }

    free(view->rate_cache);
    free(view);
}
