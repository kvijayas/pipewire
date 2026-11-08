/* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * pw-voiceui.c  —  PipeWire SVA Voice UI client utility
 *
 * Commands:
 *   list               Discover pal_source_voice_ui node(s)
 *   info               Print node registry properties
 *   set <key> <value>  Send an SVA property to the plugin
 *   listen             Persistent listener: receive detection/status
 *                      events (does not by itself save LAB PCM — see
 *                      the `replay` interactive command below)
 *   set-param          Low-level SPA param (advanced)
 *   send-command       Low-level SPA command (advanced)
 *
 * Interactive mode (listen on a TTY — auto-detected via isatty()):
 *   status   Print current detection/LAB state
 *   replay   Request cached LAB and save it as PCM
 *   restart  Send sva.restart=true
 *   reset    Send sva.reset=true
 *   quit     Exit cleanly
 *   help     Show commands
 *
 * Non-interactive (piped/scripted):
 *   listen runs without the interactive prompt; Ctrl+C exits.
 */

#include "config.h"

#include <errno.h>
#include <getopt.h>
#include <signal.h>          /* SIGINT, SIGTERM */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <unistd.h>

#include <spa/debug/file.h>
#include <spa/debug/pod.h>
#include <spa/debug/types.h>
#include <spa/param/audio/format-utils.h>
#include <spa/param/audio/raw.h>
#include <spa/param/buffers.h>
#include <spa/param/props.h>
#include <spa/pod/builder.h>
#include <spa/pod/dynamic.h>
#include <spa/pod/parser.h>
#include <spa/utils/json-pod.h>
#include <spa/utils/keys.h>
#include <spa/utils/result.h>
#include <spa/utils/string.h>

#include <pipewire/pipewire.h>

/* ─────────────────────────────────────────────────────────────── */
/*  Constants                                                       */
/* ─────────────────────────────────────────────────────────────── */
#define DEFAULT_TARGET_ROLE  "VoiceUI"
#define DEFAULT_TARGET_NAME  "pal_source_voice_ui"

/* Fixed audio contract — must match pw-pal-plugin's PAL capture format */
#define SVA_SAMPLE_RATE   16000
#define SVA_CHANNELS      1
#define SVA_BIT_WIDTH     16
#define SVA_FRAME_SIZE    2       /* channels * (bit_width / 8) */
#define SVA_LAB_DEFAULT_DURATION_MS  2000  /* must match plugin's SVA_LAB_DURATION */
#define SVA_LAB_GRACE_NSEC  200000000L     /* 200ms: time to wait for the final
                                             * data-path buffer after "complete" */

/* ─────────────────────────────────────────────────────────────── */
/*  Registry global (cached node info)                              */
/* ─────────────────────────────────────────────────────────────── */
struct global {
    struct spa_list      link;
    uint32_t             id;
    uint32_t             permissions;
    uint32_t             version;
    char                *type;
    struct pw_properties *props;
};

/* ─────────────────────────────────────────────────────────────── */
/*  Application state                                               */
/* ─────────────────────────────────────────────────────────────── */
struct data {
    /* PipeWire core objects */
    struct pw_main_loop *loop;
    struct pw_context   *context;
    struct pw_core      *core;
    struct pw_registry  *registry;
    struct pw_node      *node;          /* bound for one-shot set commands */
    struct spa_hook      core_listener;
    struct spa_hook      registry_listener;

    /* Registry cache */
    struct spa_list      globals;

    /* Target selection */
    const char   *target_name;
    const char   *target_role;
    uint32_t      target_id;
    bool          have_target_id;
    struct global *target;

    /* One-shot command mode */
    bool         list;
    bool         info;
    bool         verbose;
    const char  *command;
    const char  *param_id;
    const char  *json;
    const char  *sva_key;
    const char  *sva_value;
    int          pending;
    int          exit_code;
    bool         quit_after_sync; 
    int          quit_sync_seq;

    /* ── listen mode ── */
    bool              listen;
    bool              listening;
    struct pw_stream *stream;
    struct spa_hook   stream_listener;
    struct spa_hook   node_listener;
    bool              stream_active; 
    /* SVA state machine, as published by the plugin via sva.state
     * (idle/loaded/active/detected) — the real IDLE->LOADED->ACTIVE->
     * DETECTED position, independent of the detected/lab_receiving
     * booleans below. Empty until the first sva.state update arrives. */
    char     sva_state[16];

    /* Detection state */
    bool     detected;
    bool     capture_available;
    bool     lab_receiving;
    uint32_t confidence;
    int32_t  capture_delay_ms;    /* ms, signed — see sva.detection.capture_delay_ms */
    int32_t  capture_preamble_ms; /* ms */
    uint64_t kw_start_ms;         /* ms — see sva.detection.kw_start_ms */
    uint64_t kw_end_ms;           /* ms — see sva.detection.kw_end_ms */

    /* LAB file output */
    FILE    *lab_fp;
    char     lab_path[PATH_MAX];
    uint64_t lab_bytes;
    uint64_t lab_target_bytes;   
    uint64_t last_logged_lab_bytes;
    uint64_t last_logged_buffered_bytes;
    uint32_t lab_duration_ms;    /* last sva.lab.duration sent (0 = plugin default) */
    bool     lab_complete_pending; /* "complete" arrived but lab_bytes hasn't reached target yet */
    struct spa_source *lab_grace_timer;
    bool     lab_requested;
    bool     have_last_cycle;
    uint32_t last_confidence;
    char     last_lab_path[PATH_MAX];
    uint64_t last_lab_bytes;
    char     last_lab_reason[32];
    time_t   last_cycle_time;

    /* Audio format (negotiated from stream) */
    uint32_t sample_rate;
    uint32_t channels;
    uint32_t bit_width;

    /* Interactive mode (TTY only) */
    bool              interactive;
    struct spa_source *stdin_src;
    bool              prompt_visible; /* true if a bare "> " is the last thing printed */
};


static struct data *g_data_for_logging = NULL;

static void sva_clear_prompt(void)
{
    if (g_data_for_logging && g_data_for_logging->prompt_visible) {
        fputs("\r\x1b[K", stdout);
        g_data_for_logging->prompt_visible = false;
    }
}

static void sva_timestamp(char *buf, size_t len)
{
    struct timespec ts;
    struct tm tmv;
    unsigned ms;
    clock_gettime(CLOCK_REALTIME, &ts);
    localtime_r(&ts.tv_sec, &tmv);
    ms = (unsigned)(ts.tv_nsec / 1000000) % 1000u;
    snprintf(buf, len, "%02d:%02d:%02d.%03u",
             tmv.tm_hour, tmv.tm_min, tmv.tm_sec, ms);
}

static void sva_log_line(FILE *stream, const char *prefix, const char *fmt, va_list ap)
    __attribute__((format(printf, 3, 0)));

static void sva_log_line(FILE *stream, const char *prefix, const char *fmt, va_list ap)
{
    char ts[16];
    sva_timestamp(ts, sizeof(ts));
    sva_clear_prompt();
    fprintf(stream, "[%s] %s", ts, prefix);
    vfprintf(stream, fmt, ap);
    fputc('\n', stream);
}

static void SVA_LOG(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void SVA_ERR(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void SVA_PHASE(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void SVA_LOG(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    sva_log_line(stdout, "[SVA] ", fmt, ap);
    va_end(ap);
}

static void SVA_ERR(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    sva_log_line(stderr, "[SVA] ERROR: ", fmt, ap);
    va_end(ap);
}

static void SVA_PHASE(const char *fmt, ...)
{
    char ts[16];
    va_list ap;
    sva_timestamp(ts, sizeof(ts));
    sva_clear_prompt();
    fprintf(stdout, "\n[%s] ──── ", ts);
    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);
    fputs(" ────\n", stdout);
}

/* Reprint the "> " prompt after a batch of output, if interactive. */
static void sva_show_prompt(struct data *d)
{
    if (!d->interactive || d->prompt_visible)
        return;
    fputs("> ", stdout);
    fflush(stdout);
    d->prompt_visible = true;
}

/* ─────────────────────────────────────────────────────────────── */
/*  Registry global helpers                                         */
/* ─────────────────────────────────────────────────────────────── */
static void global_free(struct global *g)
{
    if (!g) return;
    spa_list_remove(&g->link);
    pw_properties_free(g->props);
    free(g->type);
    free(g);
}

static const char *global_prop(struct global *g, const char *key)
{
    return g->props ? pw_properties_get(g->props, key) : NULL;
}

/*
 * Match a node as the VoiceUI target.
 * Priority:
 *   1. Explicit -i <id>
 *   2. node.name = pal_source_voice_ui  (or -n override)
 *   3. media.role = VoiceUI  AND  media.class contains "Audio/Source"
 */
static bool global_is_voiceui_node(struct data *d, struct global *g)
{
    if (!spa_streq(g->type, PW_TYPE_INTERFACE_Node))
        return false;
    if (d->have_target_id)
        return g->id == d->target_id;

    const char *name   = global_prop(g, PW_KEY_NODE_NAME);
    const char *role   = global_prop(g, PW_KEY_MEDIA_ROLE);
    const char *mclass = global_prop(g, PW_KEY_MEDIA_CLASS);

    if (d->target_name && name && spa_streq(name, d->target_name))
        return true;
    return role && spa_streq(role, d->target_role) &&
           mclass && strstr(mclass, "Audio/Source") != NULL;
}

/* ─────────────────────────────────────────────────────────────── */
/*  Property validation                                             */
/* ─────────────────────────────────────────────────────────────── */
static bool parse_bool_value(const char *value, bool *result)
{
    if (spa_streq(value, "true") || spa_streq(value, "1") ||
        spa_streq(value, "yes")) {
        *result = true;  return true;
    }
    if (spa_streq(value, "false") || spa_streq(value, "0") ||
        spa_streq(value, "no")) {
        *result = false; return true;
    }
    return false;
}

static const char *wire_key(const char *key)
{
    if (spa_streq(key, "sva.model.path"))
        return "sva.sound.model.path";
    return key;
}

static bool validate_sva_property(const char *key, const char *value)
{
    char *end;
    long lv;
    bool bv;
    const char *wk = wire_key(key);

    if (spa_streq(wk, "sva.confidence.level")) {
        errno = 0;
        lv = strtol(value, &end, 10);

        if (errno || *end || lv < 0 || lv > 100) {
            SVA_ERR("sva.confidence.level must be an integer in range 0..100");
            return false;
        }
        return true;
    }

    if (spa_streq(wk, "sva.sound.model.path")) {
        if (value[0] == '\0') {
            SVA_ERR("sva.model.path must not be empty");
            return false;
        }
        return true;
    }

    if (spa_streq(wk, "sva.lab.enabled") ||
        spa_streq(wk, "sva.reset") ||
        spa_streq(wk, "sva.restart") ||
        spa_streq(wk, "sva.lab.acquire")) {

        if (!parse_bool_value(value, &bv)) {
            SVA_ERR("%s must be true/false", key);
            return false;
        }
        return true;
    }

    if (spa_streq(wk, "sva.mode")) {
        if (!spa_streq(value, "LPI") &&
            !spa_streq(value, "NLPI")) {

            SVA_ERR("sva.mode must be LPI or NLPI");
            return false;
        }
        return true;
    }

    if (spa_streq(wk, "sva.lab.duration")) {
        errno = 0;
        lv = strtol(value, &end, 10);

        if (errno || *end || lv <= 0) {
            SVA_ERR("sva.lab.duration must be greater than 0");
            return false;
        }
        return true;
    }

    SVA_ERR("unsupported SVA key '%s'", key);

    fprintf(stderr,
        "  Supported keys: sva.confidence.level, sva.model.path,\n"
        "                  sva.sound.model.path, sva.mode,\n"
        "                  sva.lab.enabled, sva.lab.duration,\n"
        "                  sva.reset, sva.restart, sva.lab.acquire\n");

    return false;
}

/* ─────────────────────────────────────────────────────────────── */
/*  SPA POD builder — typed SVA properties                          */
/* ─────────────────────────────────────────────────────────────── */
/*
 * Sends typed SPA values that match what the plugin publishes and
 * what the plugin's parser expects:
 *   integers -> spa_pod_builder_int()
 *   booleans -> spa_pod_builder_bool()
 *   strings  -> spa_pod_builder_string()
 */
static bool build_sva_props_pod(struct spa_pod_builder *b,
                                const char *key,
                                const char *value,
                                struct spa_pod **pod)
{
    struct spa_pod_frame frame[2];
    const char *wk = wire_key(key);

    spa_pod_builder_push_object(b, &frame[0],
        SPA_TYPE_OBJECT_Props, SPA_PARAM_Props);
    spa_pod_builder_prop(b, SPA_PROP_params, 0);
    spa_pod_builder_push_struct(b, &frame[1]);
    spa_pod_builder_string(b, wk);

    if (spa_streq(wk, "sva.confidence.level") ||
        spa_streq(wk, "sva.lab.duration")) {
        spa_pod_builder_int(b, atoi(value));
    } else if (spa_streq(wk, "sva.lab.enabled") ||
               spa_streq(wk, "sva.reset")       ||
               spa_streq(wk, "sva.restart")     ||
               spa_streq(wk, "sva.lab.acquire")) {
        bool bv = false;
        parse_bool_value(value, &bv);
        spa_pod_builder_bool(b, bv);
    } else {
        /* string: model path, mode, etc. */
        spa_pod_builder_string(b, value);
    }

    spa_pod_builder_pop(b, &frame[1]);
    *pod = spa_pod_builder_pop(b, &frame[0]);
    return (*pod != NULL);
}

/* ─────────────────────────────────────────────────────────────── */
/*  Print helpers                                                   */
/* ─────────────────────────────────────────────────────────────── */
static void print_global(struct global *g)
{
    const char *name   = global_prop(g, PW_KEY_NODE_NAME);
    const char *desc   = global_prop(g, PW_KEY_NODE_DESCRIPTION);
    const char *mclass = global_prop(g, PW_KEY_MEDIA_CLASS);
    const char *role   = global_prop(g, PW_KEY_MEDIA_ROLE);

    printf("  id          : %u\n", g->id);
    if (name)   printf("  node.name   : %s\n", name);
    if (desc)   printf("  description : %s\n", desc);
    if (mclass) printf("  media.class : %s\n", mclass);
    if (role)   printf("  media.role  : %s\n", role);
}

static void print_usage(const char *name)
{
    fprintf(stderr,
        "Usage:\n"
        "  %s [options] list\n"
        "  %s [options] info\n"
        "  %s [options] listen\n"
        "  %s [options] set <sva-key> <value>\n"
        "  %s [options] set-param <param-id> <param-json>\n"
        "  %s [options] send-command <command-id> <command-json>\n"
        "\n"
        "Options:\n"
        "  -n, --name <node-name>   Target node.name  (default: %s)\n"
        "  -r, --role <media-role>  Target media.role (default: %s)\n"
        "  -i, --id <node-id>       Target explicit PipeWire node id\n"
        "  -v, --verbose            Print the POD being sent\n"
        "  -h, --help               Show this help\n"
        "\n"
        "SVA keys for 'set':\n"
        "  sva.model.path / sva.sound.model.path   string\n"
        "  sva.confidence.level                    int   0..100\n"
        "  sva.mode                                string  LPI|NLPI\n"
        "  sva.lab.enabled                         bool\n"
        "  sva.lab.duration                        int   (ms)\n"
        "  sva.reset                               bool\n"
        "  sva.restart                             bool\n"
        "  sva.lab.acquire                         bool\n",
        name, name, name, name, name, name,
        DEFAULT_TARGET_NAME, DEFAULT_TARGET_ROLE);
}

static void lab_compute_target(struct data *d)
{
    uint32_t rate   = d->sample_rate ? d->sample_rate : SVA_SAMPLE_RATE;
    uint32_t chans  = d->channels    ? d->channels    : SVA_CHANNELS;
    uint32_t bits   = d->bit_width   ? d->bit_width   : SVA_BIT_WIDTH;
    uint32_t dur_ms = d->lab_duration_ms ? d->lab_duration_ms : SVA_LAB_DEFAULT_DURATION_MS;
    uint32_t bytes_per_ms = (rate * chans * (bits / 8)) / 1000;
    d->lab_target_bytes = (uint64_t)bytes_per_ms * dur_ms;
}

static void lab_grace_timer_disarm(struct data *d)
{
    if (d->lab_grace_timer) {
        struct timespec none = { 0, 0 };
        pw_loop_update_timer(pw_main_loop_get_loop(d->loop),
                              d->lab_grace_timer, &none, NULL, false);
    }
    d->lab_complete_pending = false;
}

static void lab_open(struct data *d)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    snprintf(d->lab_path, sizeof(d->lab_path),
             "/tmp/pw_voiceui_lab_%ld_%09ld.pcm",
             (long)ts.tv_sec, ts.tv_nsec);
    d->lab_fp = fopen(d->lab_path, "wb");
    if (d->lab_fp) {
        d->lab_bytes     = 0;
        d->lab_receiving = true;            /* set AFTER confirming open */
        if (d->lab_target_bytes == 0)
            lab_compute_target(d);
        lab_grace_timer_disarm(d);
        SVA_LOG("LAB file opened  : %s", d->lab_path);
        SVA_LOG("               Format   : S16LE");
        SVA_LOG("               Rate     : %u Hz",
                d->sample_rate ? d->sample_rate : SVA_SAMPLE_RATE);
        SVA_LOG("               Channels : %u",
                d->channels ? d->channels : SVA_CHANNELS);
        SVA_LOG("               Expected : %u ms (%llu bytes) — wait for "
                "\"LAB COMPLETE\" before reading the file",
                d->lab_duration_ms ? d->lab_duration_ms : SVA_LAB_DEFAULT_DURATION_MS,
                (unsigned long long)d->lab_target_bytes);
    } else {
        d->lab_receiving = false;
        SVA_ERR("Failed to open LAB file %s: %m", d->lab_path);
    }
}

static void lab_close(struct data *d, const char *reason)
{
    bool was_open = d->lab_fp != NULL;
    d->lab_requested = false; 
    lab_grace_timer_disarm(d);
    if (d->lab_fp) {
        fflush(d->lab_fp);
        fclose(d->lab_fp);
        d->lab_fp = NULL;
    }
    if (was_open && d->lab_path[0]) {
        d->have_last_cycle    = true;
        d->last_confidence    = d->confidence;
        d->last_lab_bytes     = d->lab_bytes;
        d->last_cycle_time    = time(NULL);
        snprintf(d->last_lab_path, sizeof(d->last_lab_path), "%s", d->lab_path);
        snprintf(d->last_lab_reason, sizeof(d->last_lab_reason), "%s", reason);
    }
   
    d->lab_receiving = false;
    d->capture_available = false;
    d->lab_target_bytes = 0;
    if (!was_open || !d->lab_path[0])
        return;
    if (spa_streq(reason, "complete")) {
        SVA_PHASE("LAB COMPLETE");
        SVA_LOG("File      : %s", d->lab_path);
        SVA_LOG("Bytes rcvd: %llu", (unsigned long long)d->lab_bytes);
        SVA_LOG("Verify    : aplay -f S16_LE -r %u -c %u %s",
                d->sample_rate ? d->sample_rate : SVA_SAMPLE_RATE,
                d->channels ? d->channels : SVA_CHANNELS, d->lab_path);
       
        SVA_LOG("LAB cache exhausted — stop any other connected capture "
                "client now (e.g. Ctrl+C pw-record); ");
    } else {
        SVA_LOG("LAB file closed (%s): %s [%llu bytes]", reason,
                d->lab_path, (unsigned long long)d->lab_bytes);
    }
}


static void lab_grace_expired(void *data, uint64_t expirations)
{
    struct data *d = data;
    (void)expirations;
    if (!d->lab_complete_pending)
        return;
    SVA_ERR("LAB grace timeout — closing with %llu/%llu bytes",
            (unsigned long long)d->lab_bytes,
            (unsigned long long)d->lab_target_bytes);
    lab_close(d, "complete");
}
/* ─────────────────────────────────────────────────────────────── */
/*  Interactive stdin handler                                       */
/* ─────────────────────────────────────────────────────────────── */
static bool send_sva_bool(struct data *d, const char *key, bool val)
{
    if (!d->node) { SVA_ERR("Node not bound — cannot send command"); return false; }
    uint8_t buf[256];
    struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buf, sizeof(buf));
    struct spa_pod_frame f[2];
    spa_pod_builder_push_object(&b, &f[0], SPA_TYPE_OBJECT_Props, SPA_PARAM_Props);
    spa_pod_builder_prop(&b, SPA_PROP_params, 0);
    spa_pod_builder_push_struct(&b, &f[1]);
    spa_pod_builder_string(&b, key);
    spa_pod_builder_bool(&b, val);
    spa_pod_builder_pop(&b, &f[1]);
    struct spa_pod *pod = spa_pod_builder_pop(&b, &f[0]);
    if (!pod) { SVA_ERR("Failed to build %s property POD", key); return false; }
    int rc = pw_node_set_param(d->node, SPA_PARAM_Props, 0, pod);
    if (rc < 0) { SVA_ERR("pw_node_set_param failed for %s: %s", key, spa_strerror(rc)); return false; }
    SVA_LOG("Sent: %s=%s", key, val ? "true" : "false");
    return true;
}
static void handle_interactive_command(struct data *d, const char *line)
{
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "%s", line);
    size_t n = strlen(cmd);
    while (n > 0 && (cmd[n-1] == '\n' || cmd[n-1] == '\r' ||
                     cmd[n-1] == ' '))
        cmd[--n] = '\0';

    /* Any further output this round starts on a clean line, not
     * appended to the trailing "> " prompt. */
    sva_clear_prompt();

    if (strncmp(cmd, "set ", 4) == 0) {
        char *key = cmd + 4;
        char *value = strchr(key, ' ');
        if (!value || value == key) {
            SVA_ERR("Usage: set <sva-key> <value>");
        } else {
            *value++ = '\0';
            while (*value == ' ') value++;
            if (!validate_sva_property(key, value)) {
                SVA_ERR("Invalid SVA property");
            } else {
                uint8_t ibuf[512];
                spa_auto(spa_pod_dynamic_builder) ib = { 0 };
                struct spa_pod *ipod = NULL;
                spa_pod_dynamic_builder_init(&ib, ibuf, sizeof(ibuf), 4096);
                if (!build_sva_props_pod(&ib.b, key, value, &ipod) || !ipod) {
                    SVA_ERR("Failed to build property POD");
                } else {
                    int irc = pw_node_set_param(d->node, SPA_PARAM_Props, 0, ipod);
                    if (irc < 0)
                        SVA_ERR("pw_node_set_param failed: %s", spa_strerror(irc));
                    else {
                        SVA_LOG("Sent: %s=%s", wire_key(key), value);
                        if (spa_streq(wire_key(key), "sva.lab.duration"))
                            d->lab_duration_ms = (uint32_t)atoi(value);
                    }
                }
            }
        }
    } else if (spa_streq(cmd, "status")) {
        SVA_PHASE("Status");
        SVA_LOG("state         : %s", d->sva_state[0] ? d->sva_state : "unknown");
        if (d->detected) {
            SVA_LOG("detected      : yes");
            SVA_LOG("confidence    : %u%%", d->confidence);
            SVA_LOG("capture_avail : %s",
                    d->capture_available ? "yes" : "no");
            SVA_LOG("capture_delay : %d ms", d->capture_delay_ms);
            SVA_LOG("capture_preamble : %d ms", d->capture_preamble_ms);
        }
        if (d->lab_receiving) {
            SVA_LOG("lab_receiving : yes");
            SVA_LOG("lab_file      : %s", d->lab_path);
            if (d->lab_target_bytes > 0) {
                SVA_LOG("lab_bytes     : %llu / %llu",
                        (unsigned long long)d->lab_bytes,
                        (unsigned long long)d->lab_target_bytes);
            } else {
                SVA_LOG("lab_bytes     : %llu",
                        (unsigned long long)d->lab_bytes);
            }
        }
        if (!d->lab_receiving) {
            
            if (d->lab_target_bytes > 0)
                SVA_LOG("lab_cache     : buffered (%llu bytes ready — use "
                        "'replay' to fetch)",
                        (unsigned long long)d->lab_target_bytes);
            else
                SVA_LOG("lab_cache     : empty");
        }
        if (!d->detected && !d->lab_receiving && d->have_last_cycle) {
            char tbuf[32];
            struct tm tmv;
            localtime_r(&d->last_cycle_time, &tmv);
            strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tmv);
            SVA_LOG("last_cycle    : %s (%s) at %s", d->last_lab_reason,
                    spa_streq(d->last_lab_reason, "complete") ? "ok" : "interrupted",
                    tbuf);
            SVA_LOG("  confidence  : %u%%", d->last_confidence);
            SVA_LOG("  lab_bytes   : %llu",
                    (unsigned long long)d->last_lab_bytes);
            SVA_LOG("  lab_file    : %s", d->last_lab_path);
        }


    } else if (spa_streq(cmd, "restart")) {
        SVA_LOG("Sending sva.restart=true ...");
        send_sva_bool(d, "sva.restart", true);

    } else if (spa_streq(cmd, "reset")) {
        SVA_LOG("Sending sva.reset=true ...");
        send_sva_bool(d, "sva.reset", true);
        lab_close(d, "cancelled");
        d->detected      = false;
        d->lab_receiving = false;

    } else if (spa_streq(cmd, "replay")) {
        SVA_LOG("Sending sva.lab.acquire=true ...");
        if (send_sva_bool(d, "sva.lab.acquire", true))
            d->lab_requested = true;

    } else if (spa_streq(cmd, "quit") || spa_streq(cmd, "exit")) {
        SVA_LOG("Quitting ...");
        send_sva_bool(d, "sva.reset", true);
        lab_close(d, "user-quit");
        d->quit_after_sync = true;
        d->quit_sync_seq = pw_core_sync(d->core, PW_ID_CORE, 0);

    } else if (spa_streq(cmd, "help") || spa_streq(cmd, "?")) {
        printf("  set <key> <value> — configure SVA\n"
               "  status   — print SVA state (idle/loaded/active/detected) + live/last-cycle detail\n"
               "  restart  — send sva.restart=true\n"
               "  reset    — send sva.reset=true\n"
               "  replay   — send sva.lab.acquire=true (re-fetch cached LAB)\n"
               "  quit     — reset server state (sva.reset) and exit cleanly\n"
               "  help     — show this help\n");
    } else if (cmd[0] != '\0') {
        if (spa_streq(cmd, "listen") || spa_streq(cmd, "list") ||
            spa_streq(cmd, "info") || strncmp(cmd, "pw-voiceui", 10) == 0) {
            printf("  Already inside 'listen' — this is a live session.\n"
                   "  Open another shell for one-shot commands like 'pw-voiceui info'.\n");
        } else {
            printf("  Unknown command '%s'  (try: help)\n", cmd);
        }
    }

    if (d->interactive) {
        sva_show_prompt(d);
    }
}

static void on_stdin(void *user_data, int fd, uint32_t mask)
{
    struct data *d = user_data;
    (void)fd; (void)mask;
    char line[256];
    if (!fgets(line, sizeof(line), stdin)) {
        SVA_LOG("stdin EOF — exiting listen mode");
        lab_close(d, "eof");
        pw_main_loop_quit(d->loop);
        return;
    }
    handle_interactive_command(d, line);
}

/* ─────────────────────────────────────────────────────────────── */
/*  PipeWire stream callbacks (listen mode)                         */
/* ─────────────────────────────────────────────────────────────── */
static void stream_state_changed(void *user_data,
                                 enum pw_stream_state old,
                                 enum pw_stream_state state,
                                 const char *error)
{
    struct data *d = user_data;
    (void)old;

    switch (state) {
    case PW_STREAM_STATE_CONNECTING:
        SVA_LOG("Stream connecting to %s ...", DEFAULT_TARGET_NAME);
        break;
    case PW_STREAM_STATE_PAUSED:
        SVA_LOG("Stream paused (negotiating format)");
        break;
    case PW_STREAM_STATE_STREAMING:
        d->stream_active = true;
        SVA_PHASE("Stream ACTIVE — ready for SVA events");
        SVA_LOG("Connected to  : %s", DEFAULT_TARGET_NAME);
        SVA_LOG("Audio format  : S16LE %u Hz %u ch",
                d->sample_rate ? d->sample_rate : SVA_SAMPLE_RATE,
                d->channels    ? d->channels    : SVA_CHANNELS);
        SVA_LOG("Waiting for keyword detection ...");
        if (d->interactive) {
            SVA_LOG("Interactive mode — type 'help' for commands");
            sva_show_prompt(d);
        } else {
            SVA_LOG("Non-interactive mode — Ctrl+C to quit");
        }
        break;
    case PW_STREAM_STATE_ERROR:
        SVA_ERR("Stream error: %s", error ? error : "(unknown)");
        d->exit_code = EXIT_FAILURE;
        pw_main_loop_quit(d->loop);
        break;
    case PW_STREAM_STATE_UNCONNECTED:
        SVA_LOG("Stream disconnected");
        if (d->listening)
            pw_main_loop_quit(d->loop);
        break;
    default:
        break;
    }
}

static void parse_sva_props(struct data *d, uint32_t id,
                            const struct spa_pod *param)
{
    if (id != SPA_PARAM_Props || !param)
        return;

    struct spa_pod_object *obj = (struct spa_pod_object *)param;
    struct spa_pod_prop   *prop;

    SPA_POD_OBJECT_FOREACH(obj, prop) {
        if (prop->key != SPA_PROP_params)
            continue;
        const struct spa_pod *params_pod = &prop->value;
        if (!spa_pod_is_struct(params_pod))
            continue;

        const struct spa_pod *iter = SPA_POD_STRUCT_BODY(params_pod);
        const struct spa_pod *end  = SPA_PTROFF(params_pod, SPA_POD_SIZE(params_pod),
                                                 const struct spa_pod);

        while (iter < end) {
            const char *key;
            if (spa_pod_get_string(iter, &key) < 0)
                break;
            iter = spa_pod_next(iter);
            if (iter >= end)
                break;
            const struct spa_pod *val = iter;
            iter = spa_pod_next(iter);

            /* ── State machine ── */
            if (spa_streq(key, "sva.state")) {
                const char *sv;
                if (spa_pod_get_string(val, &sv) == 0)
                    snprintf(d->sva_state, sizeof(d->sva_state), "%s", sv);

           
            } else if (spa_streq(key, "sva.detection.status")) {
                const char *sv;
                if (spa_pod_get_string(val, &sv) == 0) {
                    if (spa_streq(sv, "detected")) {
                        bool is_new = !d->detected;
                        d->detected = true;
                        if (is_new) {
                            d->last_logged_lab_bytes = 0;
                            d->last_logged_buffered_bytes = 0;
                            SVA_PHASE("KEYWORD DETECTED!");
                            SVA_LOG("Confidence        : %u%%", d->confidence);
                            SVA_LOG("KW Start          : %llu ms", (unsigned long long)d->kw_start_ms);
                            SVA_LOG("KW End            : %llu ms", (unsigned long long)d->kw_end_ms);
                            SVA_LOG("Capture available : %s",
                                    d->capture_available ? "YES" : "no");
                        }
                    } else if (spa_streq(sv, "idle")) {
                        d->detected = false;
                        d->capture_available = false;
                    }
                }

            } else if (spa_streq(key, "sva.detection.confidence")) {
                int32_t iv;
                const char *sv;
                if (spa_pod_get_int(val, &iv) == 0)
                    d->confidence = (uint32_t)iv;
                else if (spa_pod_get_string(val, &sv) == 0)
                    d->confidence = (uint32_t)atoi(sv);

            } else if (spa_streq(key, "sva.lab.duration")) {
                int32_t iv;
                const char *sv;
                if (spa_pod_get_int(val, &iv) == 0 && iv > 0)
                    d->lab_duration_ms = (uint32_t)iv;
                else if (spa_pod_get_string(val, &sv) == 0 && atoi(sv) > 0)
                    d->lab_duration_ms = (uint32_t)atoi(sv);

            } else if (spa_streq(key, "sva.lab.cached_bytes")) {
                int64_t lv;
                const char *sv;
                if (spa_pod_get_long(val, &lv) == 0 && lv >= 0) {
                    d->lab_target_bytes = (uint64_t)lv;
                } else if (spa_pod_get_string(val, &sv) == 0) {
                    char *end;
                    unsigned long long value = strtoull(sv, &end, 10);
                    if (*sv && *end == '\0')
                        d->lab_target_bytes = (uint64_t)value;
                }

            } else if (spa_streq(key, "sva.detection.capture_available")) {
                bool bv;
                const char *sv;
                if (spa_pod_get_bool(val, &bv) == 0)
                    d->capture_available = bv;
                else if (spa_pod_get_string(val, &sv) == 0)
                    d->capture_available = spa_streq(sv, "true");

            } else if (spa_streq(key, "sva.detection.capture_delay_ms")) {
                int32_t iv;
                const char *sv;
                if (spa_pod_get_int(val, &iv) == 0)
                    d->capture_delay_ms = iv;
                else if (spa_pod_get_string(val, &sv) == 0)
                    d->capture_delay_ms = atoi(sv);

            } else if (spa_streq(key, "sva.detection.capture_preamble_ms")) {
                int32_t iv;
                const char *sv;
                if (spa_pod_get_int(val, &iv) == 0)
                    d->capture_preamble_ms = iv;
                else if (spa_pod_get_string(val, &sv) == 0)
                    d->capture_preamble_ms = atoi(sv);

            } else if (spa_streq(key, "sva.detection.kw_start_ms")) {
                int64_t lv;
                const char *sv;
                if (spa_pod_get_long(val, &lv) == 0 && lv >= 0)
                    d->kw_start_ms = (uint64_t)lv;
                else if (spa_pod_get_string(val, &sv) == 0)
                    d->kw_start_ms = strtoull(sv, NULL, 10);

            } else if (spa_streq(key, "sva.detection.kw_end_ms")) {
                int64_t lv;
                const char *sv;
                if (spa_pod_get_long(val, &lv) == 0 && lv >= 0)
                    d->kw_end_ms = (uint64_t)lv;
                else if (spa_pod_get_string(val, &sv) == 0)
                    d->kw_end_ms = strtoull(sv, NULL, 10);

            } else if (spa_streq(key, "sva.lab.status")) {
                const char *sv;
                if (spa_pod_get_string(val, &sv) == 0) {
                    if (spa_streq(sv, "delivering")) {
                        if (!d->lab_requested) {
                            SVA_LOG("LAB status: delivering — ignoring "
                                    "(not requested via 'replay' on this session)");
                        } else if (!d->lab_fp) {
                            SVA_LOG("LAB status: delivering — PCM buffers incoming");
                            SVA_PHASE("LAB RECEIVING");
                            lab_open(d);
                            d->lab_requested = false; /* one-shot: consumed */
                        }
                    } else if (spa_streq(sv, "complete")) {
                        SVA_LOG("LAB status: complete");
                        if (d->lab_fp && d->lab_target_bytes > 0 &&
                            d->lab_bytes < d->lab_target_bytes) {
                            d->lab_complete_pending = true;
                            if (d->lab_grace_timer) {
                                struct timespec timeout =
                                    { 0, SVA_LAB_GRACE_NSEC };
                                pw_loop_update_timer(
                                    pw_main_loop_get_loop(d->loop),
                                    d->lab_grace_timer, &timeout, NULL, false);
                            }
                        } else {
                            lab_close(d, "complete");
                        }
                    } else if (spa_streq(sv, "error")) {
                        SVA_ERR("LAB status: error reported by plugin");
                        lab_close(d, "error");
                    } else if (spa_streq(sv, "idle")) {
                        lab_close(d, "idle-sync");
                    } else if (spa_streq(sv, "busy")) {
                        d->lab_requested = false;
                        SVA_LOG("LAB status: a drain or delivery is already "
                                "in progress — try 'replay' again shortly");
                    } else if (spa_streq(sv, "buffered")) {
                        if (d->lab_target_bytes != d->last_logged_buffered_bytes) {
                            SVA_LOG("LAB status: buffered (%llu bytes ready — "
                                    "use 'replay' to fetch)",
                                    (unsigned long long)d->lab_target_bytes);
                            d->last_logged_buffered_bytes = d->lab_target_bytes;
                        }
                    }
                }

            } else if (spa_streq(key, "sva.lab.bytes")) {
                int64_t lv;
                const char *sv;
                uint64_t plugin_bytes = 0;
                if (spa_pod_get_long(val, &lv) == 0)
                    plugin_bytes = (uint64_t)lv;
                else if (spa_pod_get_string(val, &sv) == 0)
                    plugin_bytes = (uint64_t)strtoull(sv, NULL, 10);
                if (plugin_bytes > 0 && plugin_bytes != d->last_logged_lab_bytes) {
                    SVA_LOG("Plugin LAB bytes  : %llu",
                            (unsigned long long)plugin_bytes);
                    d->last_logged_lab_bytes = plugin_bytes;
                }
            }
        }
    }
}

static void stream_param_changed(void *user_data, uint32_t id,
                                 const struct spa_pod *param)
{
    (void)user_data;
    (void)id;
    (void)param;
}

static void node_param_changed(void *data, int seq, uint32_t id,
                               uint32_t index, uint32_t next,
                               const struct spa_pod *param)
{
    struct data *d = data;
    (void)seq;
    (void)index;
    (void)next;

    if (!d->stream_active)
        return;

    parse_sva_props(d, id, param);
    if (d->interactive && d->listening)
        sva_show_prompt(d);
}

static const struct pw_node_events node_events = {
    PW_VERSION_NODE_EVENTS,
    .param = node_param_changed,
};

/*
 * stream_process — receives LAB PCM buffers from the plugin.
 * Called by the PipeWire graph thread.
 * Always returns the dequeued buffer, even when empty or not writing.
 */
static void stream_process(void *user_data)
{
    struct data *d = user_data;
    struct pw_buffer *pwbuf;
    struct spa_buffer *buf;
    struct spa_data *sd;
    const void *src;
    uint32_t off;
    uint32_t size;
    size_t written;
    int rc;

    pwbuf = pw_stream_dequeue_buffer(d->stream);
    if (pwbuf == NULL) {
        SVA_ERR("pw_stream_dequeue_buffer returned NULL");
        return;
    }

    buf = pwbuf->buffer;
    if (buf == NULL || buf->n_datas == 0)
        goto queue_buffer;

    sd = &buf->datas[0];

    if (sd->data == NULL || sd->chunk == NULL)
        goto queue_buffer;

   
    off = SPA_MIN(sd->chunk->offset, sd->maxsize);
    size = SPA_MIN(sd->chunk->size, sd->maxsize - off);

    if (size == 0)
        goto queue_buffer;

    
    if (d->lab_fp == NULL || !d->lab_receiving)
        goto queue_buffer;

    
    if (d->lab_target_bytes > 0 && d->lab_bytes >= d->lab_target_bytes)
        goto queue_buffer;
    if (d->lab_target_bytes > 0 && d->lab_bytes + size > d->lab_target_bytes)
        size = (uint32_t)(d->lab_target_bytes - d->lab_bytes);
    if (size == 0)
        goto queue_buffer;

    src = SPA_PTROFF(sd->data, off, const void);

    written = fwrite(src, 1, size, d->lab_fp);
    if (written != size) {
        SVA_ERR("LAB fwrite failed: wrote %zu of %u bytes: %s",
                written, size, strerror(errno));

        fflush(d->lab_fp);
        fclose(d->lab_fp);
        d->lab_fp = NULL;
        d->lab_receiving = false;
        goto queue_buffer;
    }

    d->lab_bytes += written;

    
    if (d->lab_complete_pending && d->lab_target_bytes > 0 &&
        d->lab_bytes >= d->lab_target_bytes)
        lab_close(d, "complete");

queue_buffer:
    rc = pw_stream_queue_buffer(d->stream, pwbuf);
    if (rc < 0) {
        SVA_ERR("pw_stream_queue_buffer failed: %s", spa_strerror(rc));
    }
}

static const struct pw_stream_events stream_events = {
    PW_VERSION_STREAM_EVENTS,
    .state_changed = stream_state_changed,
    .param_changed = stream_param_changed,
    .process       = stream_process,
};

/* ─────────────────────────────────────────────────────────────── */
/*  Core / registry callbacks                                       */
/* ─────────────────────────────────────────────────────────────── */
static void core_done(void *user_data, uint32_t id, int seq)
{
    struct data *d = user_data;
    (void)id;
    if (d->quit_after_sync && seq == d->quit_sync_seq) {
        pw_main_loop_quit(d->loop);
        return;
    }
    if (!d->listening && d->pending == seq)
        pw_main_loop_quit(d->loop);
}

static void core_error(void *user_data, uint32_t id, int seq,
                       int res, const char *message)
{
    struct data *d = user_data;
    fprintf(stderr, "PipeWire error id:%u seq:%d res:%d (%s): %s\n",
            id, seq, res, spa_strerror(res), message);
    d->exit_code = EXIT_FAILURE;
    if (d->listening) {
        SVA_ERR("Fatal PipeWire core error during listen; stopping");
        pw_main_loop_quit(d->loop);
        return;
    }
    if (seq == d->pending || d->pending == 0)
        pw_main_loop_quit(d->loop);
}

static const struct pw_core_events core_events = {
    PW_VERSION_CORE_EVENTS,
    .done  = core_done,
    .error = core_error,
};

static void registry_global(void *user_data, uint32_t id,
                             uint32_t permissions, const char *type,
                             uint32_t version,
                             const struct spa_dict *props)
{
    struct data *d = user_data;
    struct global *g;

    g = calloc(1, sizeof(*g));
    if (!g) {
        d->exit_code = EXIT_FAILURE;
        pw_main_loop_quit(d->loop);
        return;
    }
    g->id          = id;
    g->permissions = permissions;
    g->version     = version;
    g->type        = strdup(type);
    g->props       = props ? pw_properties_new_dict(props) : NULL;
    spa_list_append(&d->globals, &g->link);

    if (global_is_voiceui_node(d, g)) {
        if (d->list) {
            SVA_PHASE("VoiceUI Node Found");
            print_global(g);
        }
        if (d->target == NULL)
            d->target = g;
    }
}

static void registry_global_remove(void *user_data, uint32_t id)
{
    struct data *d = user_data;
    struct global *g, *tmp;
    spa_list_for_each_safe(g, tmp, &d->globals, link) {
        if (g->id == id) {
            if (d->target == g)
                d->target = NULL;
            global_free(g);
            return;
        }
    }
}

static const struct pw_registry_events registry_events = {
    PW_VERSION_REGISTRY_EVENTS,
    .global        = registry_global,
    .global_remove = registry_global_remove,
};

static bool parse_pod(const char *type_name, const char *json,
                      const struct spa_type_info *types,
                      struct spa_pod_dynamic_builder *builder,
                      struct spa_pod **pod)
{
    const struct spa_type_info *ti;
    struct spa_error_location loc;

    ti = spa_debug_type_find_short(types, type_name);
    if (!ti) {
        fprintf(stderr, "unknown type '%s'\n", type_name);
        return false;
    }
    int res = spa_json_to_pod_checked(&builder->b, 0, ti,
                                      json, strlen(json), &loc);
    if (res < 0) {
        if (loc.line != 0)
            spa_debug_file_error_location(stderr, &loc,
                "syntax error in json '%s': %s", json, loc.reason);
        fprintf(stderr, "can't make POD for '%s': %s\n",
                type_name,
                loc.reason ? loc.reason : spa_strerror(res));
        return false;
    }
    *pod = spa_pod_builder_deref(&builder->b, 0);
    if (!*pod) {
        fprintf(stderr, "can't dereference built POD\n");
        return false;
    }
    return true;
}

/* ─────────────────────────────────────────────────────────────── */
/*  Action runner                                                   */
/* ─────────────────────────────────────────────────────────────── */
static bool bind_target(struct data *d)
{
    if (!d->target) {
        SVA_ERR("VoiceUI node not found");
        return false;
    }
    d->node = pw_registry_bind(d->registry, d->target->id,
                               PW_TYPE_INTERFACE_Node,
                               PW_VERSION_NODE, 0);
    if (!d->node) {
        SVA_ERR("can't bind VoiceUI node id:%u: %m", d->target->id);
        return false;
    }
    return true;
}

static bool run_action(struct data *d)
{
    /* list: nodes already printed during registry scan */
    if (d->list)
        return true;

    if (!d->target) {
        SVA_ERR("VoiceUI node not found (is the plugin loaded?)");
        return false;
    }

    /* info */
    if (d->info) {
        SVA_PHASE("Node Info");
        print_global(d->target);
        SVA_LOG("(Runtime SVA state is available via plugin logs)");
        return true;
    }

    /* ── listen mode ── */
    if (d->listen) {
        SVA_PHASE("Entering Listen Mode");
        SVA_LOG("Targeting node: %s (id %u)",
                DEFAULT_TARGET_NAME, d->target->id);
        SVA_LOG("Requesting    : S16LE %u Hz %u ch",
                SVA_SAMPLE_RATE, SVA_CHANNELS);

        /* Bind node for interactive restart/reset commands */
        if (!bind_target(d))
            return false;

        pw_node_add_listener(d->node, &d->node_listener, &node_events, d);
        {
            uint32_t ids[1] = { SPA_PARAM_Props };
            pw_node_subscribe_params(d->node, ids, 1);
        }

        /*
         * Create listener stream.
         * Properties:
         *   PW_KEY_MEDIA_TYPE     = "Audio"
         *   PW_KEY_MEDIA_CATEGORY = "Capture"
         *   PW_KEY_MEDIA_ROLE     = "VoiceUI"
         *
         * Do NOT free sp after pw_stream_new() — ownership transfers.
         * Only free it if pw_stream_new() fails.
         */
        struct pw_properties *sp =
            pw_properties_new(
                PW_KEY_NODE_NAME,        "pw-voiceui-listener",
                PW_KEY_MEDIA_TYPE,       "Audio",
                PW_KEY_MEDIA_CATEGORY,   "Capture",
                PW_KEY_MEDIA_ROLE,       "VoiceUI",
                NULL);
        if (!sp) {
            SVA_ERR("pw_properties_new failed: %m");
            return false;
        }

        d->stream = pw_stream_new(d->core, "pw-voiceui", sp);
        if (!d->stream) {
            /* pw_stream_new failed — sp NOT consumed, must free */
            pw_properties_free(sp);
            SVA_ERR("pw_stream_new failed: %m");
            return false;
        }
        /* sp is now owned by d->stream — do NOT free sp */

        pw_stream_add_listener(d->stream, &d->stream_listener,
                               &stream_events, d);

        uint8_t buf[256];
        struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buf, sizeof(buf));
        const struct spa_pod *params[1];
        params[0] = spa_format_audio_raw_build(&b, SPA_PARAM_EnumFormat,
            &SPA_AUDIO_INFO_RAW_INIT(
                .format   = SPA_AUDIO_FORMAT_S16_LE,
                .rate     = SVA_SAMPLE_RATE,
                .channels = SVA_CHANNELS));

        d->sample_rate = SVA_SAMPLE_RATE;
        d->channels    = SVA_CHANNELS;
        d->bit_width   = SVA_BIT_WIDTH;

        /*
         * Connect to the discovered target node ID directly.
         * Using d->target->id ensures -n and -i options work correctly
         * rather than relying on PW_ID_ANY + PW_KEY_TARGET_OBJECT.
         */
        int rc = pw_stream_connect(d->stream,
                                   PW_DIRECTION_INPUT,
                                   d->target->id,
                                   PW_STREAM_FLAG_AUTOCONNECT |
                                   PW_STREAM_FLAG_MAP_BUFFERS |
                                   PW_STREAM_FLAG_NO_CONVERT,
                                   params, 1);
        if (rc < 0) {
            SVA_ERR("pw_stream_connect failed: %s", spa_strerror(rc));
            pw_stream_destroy(d->stream);
            d->stream = NULL;
            return false;
        }

        SVA_LOG("Stream connecting — waiting for graph scheduling ...");

        /* Register stdin watcher for interactive TTY mode */
        if (d->interactive) {
            struct pw_loop *pwloop = pw_main_loop_get_loop(d->loop);
            d->stdin_src = pw_loop_add_io(pwloop, STDIN_FILENO,
                                          SPA_IO_IN, false,
                                          on_stdin, d);
        }

        d->lab_grace_timer = pw_loop_add_timer(pw_main_loop_get_loop(d->loop),
                                               lab_grace_expired, d);
        if (!d->lab_grace_timer) {
            SVA_ERR("failed to create LAB grace timer");
            if (d->stdin_src) {
                pw_loop_destroy_source(pw_main_loop_get_loop(d->loop),
                                       d->stdin_src);
                d->stdin_src = NULL;
            }
            pw_stream_destroy(d->stream);
            d->stream = NULL;
            return false;
        }

        d->listening = true;
        pw_main_loop_run(d->loop);

        /* Cleanup after loop exit */
        lab_close(d, "exit");
        if (d->lab_grace_timer) {
            pw_loop_destroy_source(pw_main_loop_get_loop(d->loop),
                                   d->lab_grace_timer);
            d->lab_grace_timer = NULL;
        }
        if (d->stdin_src) {
            pw_loop_destroy_source(pw_main_loop_get_loop(d->loop),
                                   d->stdin_src);
            d->stdin_src = NULL;
        }
        if (d->stream) {
            pw_stream_destroy(d->stream);
            d->stream = NULL;
        }
        return d->exit_code == EXIT_SUCCESS;
    }

    /* ── one-shot set / set-param / send-command ── */
    if (!bind_target(d))
        return false;

    uint8_t buffer[1024];
    spa_auto(spa_pod_dynamic_builder) builder = { 0 };
    struct spa_pod *pod;
    spa_pod_dynamic_builder_init(&builder, buffer, sizeof(buffer), 4096);

    if (spa_streq(d->command, "set")) {
        if (!build_sva_props_pod(&builder.b, d->sva_key, d->sva_value, &pod)) {
            SVA_ERR("can't build SVA Props POD");
            return false;
        }
        if (d->verbose)
            spa_debug_pod(0, NULL, pod);
        int rc = pw_node_set_param(
                d->node,
                SPA_PARAM_Props,
                0,
                pod);

            if (rc < 0) {
                SVA_ERR("pw_node_set_param failed for %s: %s",
                        wire_key(d->sva_key),
                        spa_strerror(rc));
                return false;
            }

        /* User-friendly confirmation */
        const char *wk = wire_key(d->sva_key);
        if (spa_streq(wk, "sva.sound.model.path")) {
            SVA_LOG("Sent: model path = %s", d->sva_value);
            SVA_LOG("Plugin will load model and start recognition");
        } else if (spa_streq(wk, "sva.reset")) {
            SVA_LOG("Sent: sva.reset=true — plugin returning to IDLE");
        } else if (spa_streq(wk, "sva.restart")) {
            SVA_LOG("Sent: sva.restart=true — plugin restarting recognition");
        } else {
            SVA_LOG("Sent: %s = %s", wk, d->sva_value);
        }

    } else if (spa_streq(d->command, "set-param")) {
        const struct spa_type_info *ti;
        if (!parse_pod(d->param_id, d->json, spa_type_param, &builder, &pod))
            return false;
        ti = spa_debug_type_find_short(spa_type_param, d->param_id);
        if (!ti) { SVA_ERR("Unknown parameter type: %s", d->param_id); return false; }
        if (d->verbose)
            spa_debug_pod(0, NULL, pod);
        int rc = pw_node_set_param(d->node, ti->type, 0, pod);
        if (rc < 0) { SVA_ERR("pw_node_set_param failed: %s", spa_strerror(rc)); return false; }
        SVA_LOG("set-param sent: %s", d->param_id);

    } else if (spa_streq(d->command, "send-command")) {
        if (!parse_pod(d->param_id, d->json,
                       spa_type_node_command_id, &builder, &pod))
            return false;
        if (d->verbose)
            spa_debug_pod(0, NULL, pod);
        int rc = pw_node_send_command(d->node, (struct spa_command *)pod);
        if (rc < 0) { SVA_ERR("pw_node_send_command failed: %s", spa_strerror(rc)); return false; }
        SVA_LOG("send-command sent: %s", d->param_id);

    } else {
        SVA_ERR("unknown command '%s'", d->command);
        return false;
    }

    d->pending = pw_core_sync(d->core, PW_ID_CORE, 0);
    pw_main_loop_run(d->loop);
    return d->exit_code == EXIT_SUCCESS;
}

/* ─────────────────────────────────────────────────────────────── */
/*  Signal handler                                                  */
/* ─────────────────────────────────────────────────────────────── */
static struct pw_main_loop *_g_loop = NULL;

static void on_signal(void *d, int sig)
{
    (void)d; (void)sig;
    if (_g_loop)
        pw_main_loop_quit(_g_loop);
}

/* ─────────────────────────────────────────────────────────────── */
/*  main                                                            */
/* ─────────────────────────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    struct data data = { 0 };
    struct global *g, *tmp;
    int opt;
    static const struct option long_options[] = {
        { "name",    required_argument, NULL, 'n' },
        { "role",    required_argument, NULL, 'r' },
        { "id",      required_argument, NULL, 'i' },
        { "verbose", no_argument,       NULL, 'v' },
        { "help",    no_argument,       NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };

    data.target_name = DEFAULT_TARGET_NAME;
    data.target_role = DEFAULT_TARGET_ROLE;
    data.exit_code   = EXIT_SUCCESS;
    spa_list_init(&data.globals);
    g_data_for_logging = &data;

    while ((opt = getopt_long(argc, argv, "n:r:i:vh",
                              long_options, NULL)) != -1) {
        char *end;
        switch (opt) {
        case 'n': data.target_name = optarg; break;
        case 'r': data.target_role = optarg; break;
        case 'i':
            data.target_id = (uint32_t)strtoul(optarg, &end, 10);
            if (*end) {
                fprintf(stderr, "invalid node id '%s'\n", optarg);
                return EXIT_FAILURE;
            }
            data.have_target_id = true;
            break;
        case 'v': data.verbose = true; break;
        case 'h': print_usage(argv[0]); return EXIT_SUCCESS;
        default:  print_usage(argv[0]); return EXIT_FAILURE;
        }
    }

    if (optind >= argc) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    data.command = argv[optind++];

    if (spa_streq(data.command, "list")) {
        data.list = true;
    } else if (spa_streq(data.command, "info")) {
        data.info = true;
    } else if (spa_streq(data.command, "listen")) {
        data.listen = true;
        /* Interactive only when both stdin and stdout are a TTY */
        data.interactive = isatty(STDIN_FILENO) && isatty(STDOUT_FILENO);
    } else if (spa_streq(data.command, "set")) {
        if (optind + 2 != argc) {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
        data.sva_key   = argv[optind++];
        data.sva_value = argv[optind++];
        if (!validate_sva_property(data.sva_key, data.sva_value))
            return EXIT_FAILURE;
    } else if (spa_streq(data.command, "set-param") ||
               spa_streq(data.command, "send-command")) {
        if (optind + 2 != argc) {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
        data.param_id = argv[optind++];
        data.json     = argv[optind++];
    } else {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    pw_init(&argc, &argv);

    data.loop = pw_main_loop_new(NULL);
    if (!data.loop) {
        fprintf(stderr, "can't create main loop: %m\n");
        return EXIT_FAILURE;
    }
    _g_loop = data.loop;

    struct pw_loop *pwl = pw_main_loop_get_loop(data.loop);
    struct spa_source *sigint_src  = pw_loop_add_signal(pwl, SIGINT,  on_signal, NULL);
    struct spa_source *sigterm_src = pw_loop_add_signal(pwl, SIGTERM, on_signal, NULL);

    data.context = pw_context_new(pwl, NULL, 0);
    if (!data.context) {
        SVA_ERR("can't create context: %m");
        data.exit_code = EXIT_FAILURE;
        goto out;
    }

    data.core = pw_context_connect(data.context, NULL, 0);
    if (!data.core) {
        SVA_ERR("can't connect to PipeWire: %m");
        data.exit_code = EXIT_FAILURE;
        goto out;
    }
    pw_core_add_listener(data.core, &data.core_listener,
                         &core_events, &data);

    data.registry = pw_core_get_registry(data.core,
                                          PW_VERSION_REGISTRY, 0); // nodes available
    if (!data.registry) {
        SVA_ERR("can't get registry: %m");
        data.exit_code = EXIT_FAILURE;
        goto out;
    }
    pw_registry_add_listener(data.registry, &data.registry_listener,
                             &registry_events, &data);

    /* Initial sync — discover all registered nodes */
    data.pending = pw_core_sync(data.core, PW_ID_CORE, 0);
    pw_main_loop_run(data.loop);

    if (!run_action(&data))
        data.exit_code = EXIT_FAILURE;

out:
    lab_close(&data, "exit");

    if (data.stream) {
        pw_stream_destroy(data.stream);
        data.stream = NULL;
    }
    if (data.node) {
        pw_proxy_destroy((struct pw_proxy *)data.node);
        data.node = NULL;
    }
    if (data.registry) {
        pw_proxy_destroy((struct pw_proxy *)data.registry);
        data.registry = NULL;
    }
    if (data.core) {
        pw_core_disconnect(data.core);
        data.core = NULL;
    }
    if (data.context) {
        pw_context_destroy(data.context);
        data.context = NULL;
    }
    if (sigint_src)
        pw_loop_destroy_source(pwl, sigint_src);
    if (sigterm_src)
        pw_loop_destroy_source(pwl, sigterm_src);
    if (data.loop) {
        pw_main_loop_destroy(data.loop);
        data.loop = NULL;
    }
    spa_list_for_each_safe(g, tmp, &data.globals, link)
        global_free(g);

    pw_deinit();
    return data.exit_code;
}
