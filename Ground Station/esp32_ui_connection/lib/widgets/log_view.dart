import 'package:flutter/material.dart';

// Purposes:
//  1. Show text output from the ESP-32
class LogView extends StatelessWidget {
  const LogView({super.key, required this.lines});

  final List<String> lines;

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        color: Colors.white,
        border: Border.all(color: Colors.black26),
        borderRadius: BorderRadius.circular(6),
      ),
      child: lines.isEmpty
          ? const Center(
              child: Text('— no data yet —', style: TextStyle(color: Colors.grey)),
            )
          : ListView.separated(
              reverse: true,
              padding: const EdgeInsets.all(12),
              itemBuilder: (_, i) => SelectableText(
                lines[i],
                style: const TextStyle(fontFamily: 'monospace'),
              ),
              separatorBuilder: (_, __) => const SizedBox(height: 4),
              itemCount: lines.length,
            ),
    );
  }
}
