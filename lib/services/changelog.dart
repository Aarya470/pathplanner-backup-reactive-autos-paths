import 'package:flutter/services.dart';

/// Reads the release notes in CHANGELOG.md, which is bundled with the app.
class Changelog {
  static const String assetPath = 'CHANGELOG.md';

  /// Load the bundled changelog text, or an empty string if it is missing.
  static Future<String> load(AssetBundle bundle) async {
    try {
      return await bundle.loadString(assetPath);
    } catch (_) {
      return '';
    }
  }

  /// The notes under "## [version]", without the heading. Null if there are none.
  static String? notesFor(String changelog, String version) {
    final lines = changelog.replaceAll('\r\n', '\n').split('\n');
    final List<String> notes = [];
    bool inSection = false;
    for (final line in lines) {
      if (line.startsWith('## ')) {
        if (inSection) break;
        inSection = line.substring(3).trim() == version;
        continue;
      }
      if (inSection) notes.add(line);
    }
    final text = notes.join('\n').trim();
    return text.isEmpty ? null : text;
  }
}
