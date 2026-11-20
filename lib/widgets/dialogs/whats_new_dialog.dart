import 'package:flutter/material.dart';

/// Shows the release notes for one version of the app.
class WhatsNewDialog extends StatelessWidget {
  final String version;
  final String? notes;

  const WhatsNewDialog({
    required this.version,
    required this.notes,
    super.key,
  });

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return AlertDialog(
      backgroundColor: colorScheme.surface,
      surfaceTintColor: colorScheme.surfaceTint,
      title: Text("What's new in v$version"),
      content: SizedBox(
        width: 480,
        child: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              if (notes == null)
                const Text('No notes for this version.')
              else
                for (final line in notes!.split('\n'))
                  if (line.trim().isNotEmpty) _buildLine(line.trim()),
            ],
          ),
        ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.of(context).pop(),
          child: const Text('OK'),
        ),
      ],
    );
  }

  Widget _buildLine(String line) {
    final bool bullet = line.startsWith('- ');
    final String text =
        (bullet ? line.substring(2) : line).replaceAll('**', '');
    return Padding(
      padding: const EdgeInsets.only(bottom: 6),
      child: bullet
          ? Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text('•  '),
                Expanded(child: Text(text)),
              ],
            )
          : Text(text),
    );
  }
}
