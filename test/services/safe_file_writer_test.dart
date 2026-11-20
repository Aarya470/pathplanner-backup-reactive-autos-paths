import 'package:file/memory.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/services/safe_file_writer.dart';

void main() {
  test('first write happens right away', () {
    final fs = MemoryFileSystem();
    final file = fs.file('/a.path');

    SafeFileWriter.write(file, 'one');

    expect(file.readAsStringSync(), 'one');
  });

  test('overlapping writes end with the newest content', () async {
    final fs = MemoryFileSystem();
    final file = fs.file('/b.path');

    SafeFileWriter.write(file, 'one');
    SafeFileWriter.write(file, 'two');
    final last = SafeFileWriter.write(file, 'three');

    expect(await last, true);
    expect(file.readAsStringSync(), 'three');
  });

  test('a write that keeps failing returns false instead of throwing',
      () async {
    final fs = MemoryFileSystem();
    final file = fs.file('/missing/folder/c.path');

    expect(await SafeFileWriter.write(file, 'x'), false);
  });

  test('a file can be written again after a failure', () async {
    final fs = MemoryFileSystem();
    final file = fs.file('/later/d.path');

    expect(await SafeFileWriter.write(file, 'x'), false);
    fs.directory('/later').createSync();
    expect(await SafeFileWriter.write(file, 'y'), true);
    expect(file.readAsStringSync(), 'y');
  });
}
