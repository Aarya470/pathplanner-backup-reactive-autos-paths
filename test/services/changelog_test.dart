import 'dart:io';

import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/services/changelog.dart';

void main() {
  const text =
      '# Changelog\n\n## 1.0.1\n\n- Fix one\n- New two\n\n## 1.0.0\n\n- First\n';

  test('notes for a version stop at the next version', () {
    expect(Changelog.notesFor(text, '1.0.1'), '- Fix one\n- New two');
    expect(Changelog.notesFor(text, '1.0.0'), '- First');
  });

  test('missing version has no notes', () {
    expect(Changelog.notesFor(text, '9.9.9'), isNull);
  });

  test('windows line endings work', () {
    expect(
        Changelog.notesFor(text.replaceAll('\n', '\r\n'), '1.0.0'), '- First');
  });

  test('the bundled changelog has notes for every listed version', () {
    final changelog = File('CHANGELOG.md').readAsStringSync();
    final versions = RegExp(r'^## (.+)$', multiLine: true)
        .allMatches(changelog)
        .map((m) => m.group(1)!.trim());
    expect(versions, isNotEmpty);
    for (final v in versions) {
      expect(Changelog.notesFor(changelog, v), isNotNull, reason: v);
    }
  });
}
