import 'dart:async';

import 'package:dio/dio.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../core/download.dart';
import '../core/models.dart';
import '../core/providers.dart';

class DeviceTab extends ConsumerWidget {
  const DeviceTab({super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final statusAsync = ref.watch(deviceStatusProvider);
    final sensorAsync = ref.watch(sensorModeProvider);
    final ip = ref.watch(deviceIpProvider) ?? '—';

    return RefreshIndicator(
      onRefresh: () async {
        ref.invalidate(deviceStatusProvider);
        ref.invalidate(sensorModeProvider);
      },
      child: ListView(
        padding: const EdgeInsets.all(16),
        children: [
          // ── Run logs ───────────────────────────────────────────────────
          // 放最上面：每次 run 开始和结束都要点它，而 System 那张卡片很长，
          // 摆在它下面每次都要滚到底。
          const _LogsCard(),
          const SizedBox(height: 12),
          // ── Connection ─────────────────────────────────────────────────
          _SectionCard(
            icon: Icons.wifi_rounded,
            title: 'Connection',
            children: statusAsync.when(
              data: (s) => [
                _InfoRow('Status', s.online ? 'Online' : 'Offline',
                    valueColor: s.online ? const Color(0xFF34C759) : Colors.red),
                _InfoRow('IP', ip),
                _InfoRow('State', s.rawState),
              ],
              loading: () => [const _LoadingRow()],
              error: (e, _) => [_InfoRow('Error', '$e', valueColor: Colors.red)],
            ),
          ),
          const SizedBox(height: 12),
          // ── Sensor ─────────────────────────────────────────────────────
          _SectionCard(
            icon: Icons.sensors_rounded,
            title: 'Sensor',
            children: [
              sensorAsync.when(
                data: (mode) => _InfoRow(
                  'Mode',
                  mode == 'realsense'
                      ? 'RealSense'
                      : mode == 'looper'
                          ? 'Looper'
                          : 'Unknown',
                  valueColor: mode == 'unknown' ? Colors.grey : null,
                ),
                loading: () => const _LoadingRow(),
                error: (_, __) => const _InfoRow('Mode', '—'),
              ),
            ],
          ),
          const SizedBox(height: 12),
          // ── System ─────────────────────────────────────────────────────
          const _SystemCard(),
        ],
      ),
    );
  }
}

// ── Run logs ───────────────────────────────────────────────────

class _LogsCard extends ConsumerStatefulWidget {
  const _LogsCard();

  @override
  ConsumerState<_LogsCard> createState() => _LogsCardState();
}

class _LogsCardState extends ConsumerState<_LogsCard> {
  bool _collecting = false;
  bool _busy = false;
  double? _startedAt;
  String? _note;
  Timer? _tick;

  @override
  void initState() {
    super.initState();
    _refreshStatus();
    // Collection outlives a page reload and a browser refresh, so the button has
    // to reflect the board's state rather than this widget's history.
    _tick = Timer.periodic(const Duration(seconds: 2), (_) {
      if (_collecting) setState(() {});
    });
  }

  @override
  void dispose() {
    _tick?.cancel();
    super.dispose();
  }

  Future<void> _refreshStatus() async {
    if (ref.read(baseUrlProvider) == null) return;
    try {
      final resp = await ref.read(dioProvider).get('/logs/collect/status');
      if (!mounted) return;
      setState(() {
        _collecting = resp.data['collecting'] as bool? ?? false;
        _startedAt = (resp.data['startedAt'] as num?)?.toDouble();
      });
    } catch (_) {/* offline; the Connection card already says so */}
  }

  Future<void> _start() async {
    setState(() => _busy = true);
    try {
      await ref.read(dioProvider).post('/logs/collect/start');
      await _refreshStatus();
      if (mounted) setState(() => _note = null);
    } catch (e) {
      if (mounted) setState(() => _note = 'Could not start: $e');
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  Future<void> _stop() async {
    setState(() => _busy = true);
    try {
      final resp = await ref
          .read(dioProvider)
          .post('/logs/collect/stop',
              options: Options(receiveTimeout: const Duration(seconds: 120)));
      final silent = (resp.data['nodesSilent'] as List?)?.cast<String>() ?? [];
      final files = (resp.data['contents'] as List?)?.length ?? 0;
      if (mounted) {
        setState(() => _note = '$files files'
            '${silent.isEmpty ? '' : '   no output from: ${silent.join(', ')}'}');
      }
      await _refreshStatus();
      ref.invalidate(logBundlesProvider);
    } catch (e) {
      if (mounted) setState(() => _note = 'Could not stop: $e');
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  Future<void> _delete(String name) async {
    try {
      await ref.read(dioProvider).delete('/logs/bundles/$name');
      ref.invalidate(logBundlesProvider);
    } catch (_) {}
  }

  @override
  Widget build(BuildContext context) {
    final base = ref.watch(baseUrlProvider);
    final bundles = ref.watch(logBundlesProvider);
    final elapsed = _startedAt == null
        ? null
        : (DateTime.now().millisecondsSinceEpoch / 1000 - _startedAt!);

    return _SectionCard(
      icon: Icons.receipt_long_rounded,
      title: 'Run logs',
      children: [
        Row(
          children: [
            Expanded(
              child: _collecting
                  ? FilledButton.icon(
                      onPressed: _busy ? null : _stop,
                      icon: const Icon(Icons.stop_rounded, size: 18),
                      style: FilledButton.styleFrom(backgroundColor: Colors.red),
                      label: Text(elapsed == null
                          ? 'Stop collecting'
                          : 'Stop collecting  (${_hms(elapsed)})'),
                    )
                  : FilledButton.icon(
                      onPressed: _busy ? null : _start,
                      icon: const Icon(Icons.fiber_manual_record_rounded, size: 18),
                      label: const Text('Start collecting'),
                    ),
            ),
            const SizedBox(width: 8),
            IconButton(
              tooltip: 'Refresh',
              onPressed: () {
                _refreshStatus();
                ref.invalidate(logBundlesProvider);
              },
              icon: const Icon(Icons.refresh_rounded, size: 20),
            ),
          ],
        ),
        if (_note != null)
          Padding(
            padding: const EdgeInsets.only(top: 6),
            child: Text(_note!,
                style: TextStyle(
                    fontSize: 12,
                    color: Theme.of(context).colorScheme.onSurfaceVariant)),
          ),
        const SizedBox(height: 4),
        bundles.when(
          data: (list) => list.isEmpty
              ? const _InfoRow('Bundles', 'none yet', dimmed: true)
              : Column(
                  children: [
                    // 只列最近 3 个：列表长了会把整张卡片顶下去，而实际要下载的永远是刚采的那几个。
                    for (final f in list.take(3))
                      _BundleRow(
                        entry: f,
                        onDownload: base == null
                            ? null
                            : () => downloadFile('$base/logs/bundles/${f.name}', f.name),
                        onDelete: () => _delete(f.name),
                      ),
                    if (list.length > 3)
                      _InfoRow('', '+${list.length - 3} more on device', dimmed: true),
                  ],
                ),
          loading: () => const _LoadingRow(),
          error: (_, __) => const _InfoRow('Bundles', 'unavailable', dimmed: true),
        ),
      ],
    );
  }
}

String _hms(double s) {
  final t = s.round();
  final m = t ~/ 60;
  return m > 0 ? '${m}m ${t % 60}s' : '${t}s';
}

class _BundleRow extends StatelessWidget {
  final FileEntry entry;
  final VoidCallback? onDownload;
  final VoidCallback onDelete;

  const _BundleRow(
      {required this.entry, required this.onDownload, required this.onDelete});

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          Expanded(
            child: Text(entry.name,
                overflow: TextOverflow.ellipsis,
                style: const TextStyle(fontSize: 13)),
          ),
          Text(entry.sizeLabel,
              style: TextStyle(fontSize: 12, color: cs.onSurfaceVariant)),
          IconButton(
            tooltip: 'Download to this computer',
            onPressed: onDownload,
            icon: const Icon(Icons.download_rounded, size: 18),
            visualDensity: VisualDensity.compact,
          ),
          IconButton(
            tooltip: 'Delete',
            onPressed: onDelete,
            icon: const Icon(Icons.delete_outline_rounded, size: 18),
            visualDensity: VisualDensity.compact,
          ),
        ],
      ),
    );
  }
}

// Ordered so the numbers that decide whether the stack survives come first: CPU and
// BPU are what run out, `available` is what the OOM killer acts on, and the data
// volume is the only one that can actually fill up.
List<Widget> _systemRows(SysInfo sys) {
  final m = sys.memBreakdown;
  return [
    _InfoRow(
      'CPU',
      '${sys.cpuPercent.toStringAsFixed(1)}%'
          '${sys.load1m != null ? "   load ${sys.load1m!.toStringAsFixed(1)}" : ""}',
      valueColor: sys.cpuPercent > 85 ? Colors.red : null,
    ),
    if (sys.cpuPerCore.length > 1)
      _InfoRow('  per core',
          sys.cpuPerCore.map((c) => c.toStringAsFixed(0)).join(' '),
          dimmed: true),
    if (sys.bpuPercent != null)
      _InfoRow('BPU', '${sys.bpuPercent!.toStringAsFixed(0)}%',
          valueColor: sys.bpuPercent! > 85 ? Colors.red : null),
    if (sys.gpuPercent != null)
      _InfoRow('GPU', '${sys.gpuPercent!.toStringAsFixed(1)}%',
          valueColor: sys.gpuPercent! > 85 ? Colors.red : null),
    for (final e in sys.tempsC.entries)
      _InfoRow('temp ${e.key}', '${e.value.toStringAsFixed(1)} \u00b0C',
          valueColor: e.value > 90 ? Colors.red : null),
    _InfoRow(
      'Memory',
      '${sys.memUsedGb.toStringAsFixed(1)}/${sys.memTotalGb.toStringAsFixed(1)} GB  (${sys.memPercent.toStringAsFixed(0)}%)',
      valueColor: sys.memPercent > 85 ? Colors.red : null,
    ),
    if (m != null) ...[
      _InfoRow('  available', '${m.availableMb.toStringAsFixed(0)} MB',
          valueColor: m.availableMb < 200 ? Colors.red : null),
      _InfoRow('  free / cached',
          '${m.freeMb.toStringAsFixed(0)} / ${m.cachedMb.toStringAsFixed(0)} MB',
          dimmed: true),
      if (m.shmemMb > 1)
        _InfoRow('  shared', '${m.shmemMb.toStringAsFixed(0)} MB', dimmed: true),
      _InfoRow(
        '  swap',
        m.swapTotalMb <= 0
            ? 'none'
            : '${(m.swapTotalMb - m.swapFreeMb).toStringAsFixed(0)}/${m.swapTotalMb.toStringAsFixed(0)} MB',
        dimmed: true,
      ),
      if (m.cmaTotalMb != null)
        _InfoRow('  cma (camera/BPU)',
            '${(m.cmaTotalMb! - (m.cmaFreeMb ?? 0)).toStringAsFixed(0)}/${m.cmaTotalMb!.toStringAsFixed(0)} MB',
            dimmed: true),
    ],
    if (sys.storage.isEmpty)
      _InfoRow(
        'Disk',
        '${sys.diskUsedGb.toStringAsFixed(1)}/${sys.diskTotalGb.toStringAsFixed(1)} GB  (${sys.diskPercent.toStringAsFixed(0)}%)',
        valueColor: sys.diskPercent > 90 ? Colors.red : null,
      )
    else
      for (final d in sys.storage) ...[
        _InfoRow(
          'Disk ${d.label}',
          '${d.usedGb.toStringAsFixed(1)}/${d.totalGb.toStringAsFixed(1)} GB  (${d.percent.toStringAsFixed(0)}%)',
          valueColor: d.percent > 90 ? Colors.red : null,
        ),
        _InfoRow('  path', d.path, dimmed: true),
      ],
  ];
}

// ── Section card ──────────────────────────────────────────────────────────────

class _SectionCard extends StatelessWidget {
  final IconData icon;
  final String title;
  final List<Widget> children;

  const _SectionCard({
    required this.icon,
    required this.title,
    required this.children,
  });

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 0,
      color: const Color(0xFF111A24),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(16),
        side: const BorderSide(color: Color(0xFF2A3B4D)),
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(children: [
              Icon(icon, size: 18, color: const Color(0xFF9EC7E8)),
              const SizedBox(width: 8),
              Text(title,
                  style: const TextStyle(
                      fontWeight: FontWeight.w700,
                      fontSize: 14,
                      color: Color(0xFFE8F2FF))),
            ]),
            const Divider(height: 20, color: Color(0xFF243446)),
            ...children,
          ],
        ),
      ),
    );
  }
}

// ── Info row ──────────────────────────────────────────────────────────────────

class _InfoRow extends StatelessWidget {
  final String label;
  final String value;
  final Color? valueColor;
  final bool dimmed;

  const _InfoRow(this.label, this.value, {this.valueColor, this.dimmed = false});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Expanded(
            flex: 3,
            child: Text(label,
                style: const TextStyle(
                    fontSize: 13,
                    fontWeight: FontWeight.w500,
                    color: Color(0xFF9FB0C3))),
          ),
          Expanded(
            flex: 5,
            child: Align(
              alignment: Alignment.centerRight,
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
                decoration: BoxDecoration(
                  color: const Color(0xFF1A2532),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Text(
                  value,
                  textAlign: TextAlign.right,
                  style: TextStyle(
                    fontSize: 13,
                    fontWeight: FontWeight.w600,
                    color: dimmed
                        ? const Color(0xFF77889A)
                        : (valueColor ?? const Color(0xFFE6EEF7)),
                  ),
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _LoadingRow extends StatelessWidget {
  const _LoadingRow();

  @override
  Widget build(BuildContext context) {
    return const Padding(
      padding: EdgeInsets.symmetric(vertical: 8),
      child: Center(child: SizedBox(width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2))),
    );
  }
}


/// System 卡片：默认仪表盘（一眼看六个数），点一下切到原来的逐行明细。
/// 明细里有 per-core、free/cached、cma、swap 这些排查才用的东西，平时只是噪音。
class _SystemCard extends ConsumerStatefulWidget {
  const _SystemCard();

  @override
  ConsumerState<_SystemCard> createState() => _SystemCardState();
}

class _SystemCardState extends ConsumerState<_SystemCard> {
  bool _detailed = false;

  @override
  Widget build(BuildContext context) {
    final statusAsync = ref.watch(deviceStatusProvider);
    final sysAsync = ref.watch(sysInfoProvider);

    final battery = statusAsync.maybeWhen(
      data: (s) => s.batteryVolts != null
          ? _Metric('Battery', s.batteryVolts!.toStringAsFixed(2), 'V',
              color: s.batteryVolts! < 10.2
                  ? Colors.red
                  : (s.batteryVolts! < 10.8 ? Colors.orange : null))
          : (s.battery != null
              ? _Metric('Battery', s.battery!.toStringAsFixed(0), '%',
                  color: s.battery! < 20 ? Colors.red : null)
              : const _Metric('Battery', '—', '')),
      orElse: () => const _Metric('Battery', '—', ''),
    );

    return InkWell(
      onTap: () => setState(() => _detailed = !_detailed),
      borderRadius: BorderRadius.circular(12),
      child: _SectionCard(
        icon: Icons.memory_rounded,
        title: _detailed ? 'System  ·  detail' : 'System',
        children: _detailed
            ? [
                statusAsync.when(
                  data: (s) => _InfoRow('Battery', battery.value + ' ' + battery.unit,
                      valueColor: battery.color),
                  loading: () => const _LoadingRow(),
                  error: (_, __) => const _InfoRow('Battery', '—'),
                ),
                sysAsync.when(
                  data: (sys) => Column(children: _systemRows(sys)),
                  loading: () => const _LoadingRow(),
                  error: (_, __) =>
                      const _InfoRow('System', 'unavailable', dimmed: true),
                ),
              ]
            : [
                sysAsync.when(
                  data: (sys) => _MetricGrid(metrics: _dashboardMetrics(battery, sys)),
                  loading: () => const _LoadingRow(),
                  error: (_, __) =>
                      const _InfoRow('System', 'unavailable', dimmed: true),
                ),
              ],
      ),
    );
  }
}

/// 仪表盘的六个数。温度取最高的那一路 —— 降频看的是最热的核，不是平均。
List<_Metric> _dashboardMetrics(_Metric battery, SysInfo sys) {
  final tMax = sys.tempsC.isEmpty
      ? null
      : sys.tempsC.values.reduce((a, b) => a > b ? a : b);
  return [
    battery,
    _Metric(
      'CPU',
      sys.cpuPercent.toStringAsFixed(0),
      sys.load1m != null ? '%  load ${sys.load1m!.toStringAsFixed(1)}' : '%',
      color: sys.cpuPercent > 85 ? Colors.red : null,
    ),
    _Metric('BPU', sys.bpuPercent?.toStringAsFixed(0) ?? '—',
        sys.bpuPercent == null ? '' : '%',
        color: (sys.bpuPercent ?? 0) > 85 ? Colors.red : null),
    _Metric('Temp', tMax?.toStringAsFixed(0) ?? '—', tMax == null ? '' : '\u00b0C',
        color: tMax == null
            ? null
            : (tMax > 90 ? Colors.red : (tMax > 80 ? Colors.orange : null))),
    _Metric('Memory', sys.memPercent.toStringAsFixed(0),
        '%  ${sys.memUsedGb.toStringAsFixed(1)}/${sys.memTotalGb.toStringAsFixed(1)}G',
        color: sys.memPercent > 85 ? Colors.red : null),
    _Metric('Disk', sys.diskPercent.toStringAsFixed(0),
        '%  ${sys.diskUsedGb.toStringAsFixed(0)}/${sys.diskTotalGb.toStringAsFixed(0)}G',
        color: sys.diskPercent > 90 ? Colors.red : null),
  ];
}

class _Metric {
  final String label;
  final String value;
  final String unit;
  final Color? color;
  const _Metric(this.label, this.value, this.unit, {this.color});
}

class _MetricGrid extends StatelessWidget {
  final List<_Metric> metrics;
  const _MetricGrid({required this.metrics});

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(builder: (context, box) {
      // 两列起步，宽了给三列。固定列数比 childAspectRatio 好调，数字不会被裁。
      final cols = box.maxWidth > 420 ? 3 : 2;
      final w = (box.maxWidth - (cols - 1) * 8) / cols;
      return Wrap(
        spacing: 8,
        runSpacing: 8,
        children: [
          for (final m in metrics) SizedBox(width: w, child: _MetricTile(m: m)),
        ],
      );
    });
  }
}

class _MetricTile extends StatelessWidget {
  final _Metric m;
  const _MetricTile({required this.m});

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final base = theme.textTheme.bodySmall?.color?.withOpacity(0.65);
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      decoration: BoxDecoration(
        color: theme.colorScheme.onSurface.withOpacity(0.05),
        borderRadius: BorderRadius.circular(8),
        border: m.color == null
            ? null
            : Border.all(color: m.color!.withOpacity(0.55)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(m.label,
              style: theme.textTheme.labelSmall?.copyWith(
                  color: base, letterSpacing: 0.4)),
          const SizedBox(height: 2),
          Row(
            crossAxisAlignment: CrossAxisAlignment.baseline,
            textBaseline: TextBaseline.alphabetic,
            children: [
              Text(m.value,
                  style: theme.textTheme.titleMedium?.copyWith(
                      fontWeight: FontWeight.w600,
                      fontFeatures: const [FontFeature.tabularFigures()],
                      color: m.color)),
              if (m.unit.isNotEmpty) ...[
                const SizedBox(width: 3),
                Expanded(
                  child: Text(m.unit,
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                      style: theme.textTheme.labelSmall?.copyWith(color: base)),
                ),
              ],
            ],
          ),
        ],
      ),
    );
  }
}
