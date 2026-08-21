import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../core/models.dart';
import '../core/providers.dart';

class DeviceTab extends ConsumerWidget {
  const DeviceTab({super.key});

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final statusAsync = ref.watch(deviceStatusProvider);
    final sensorAsync = ref.watch(sensorModeProvider);
    final sysAsync = ref.watch(sysInfoProvider);
    final ip = ref.watch(deviceIpProvider) ?? '—';

    return RefreshIndicator(
      onRefresh: () async {
        ref.invalidate(deviceStatusProvider);
        ref.invalidate(sensorModeProvider);
      },
      child: ListView(
        padding: const EdgeInsets.all(16),
        children: [
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
          _SectionCard(
            icon: Icons.memory_rounded,
            title: 'System',
            children: [
              statusAsync.when(
                data: (s) => s.battery != null
                    ? _InfoRow(
                        'Battery',
                        '${s.battery!.toStringAsFixed(0)}%',
                        valueColor: s.battery! < 20 ? Colors.red : null,
                      )
                    : const _InfoRow('Battery', '—'),
                loading: () => const _LoadingRow(),
                error: (_, __) => const _InfoRow('Battery', '—'),
              ),
              sysAsync.when(
                data: (sys) => Column(children: _systemRows(sys)),
                loading: () => const _LoadingRow(),
                error: (_, __) => const _InfoRow('System', 'unavailable', dimmed: true),
              ),
            ],
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
