import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

// Purpose: TCP Networking Logic
// This file does the following: 
//  1. Handles all socket networking
//  2. Connects to ESP32 over TCP
//  3. Decodes incoming bytes into text lines
//  4. Emits text lines as stream
//  5. Sends raw bytes
//  6. Handles disconnect
// 
// Keeps networking code isolated for ease of file manipulation
class TcpClient {
  Socket? _sock;
  final _linesCtrl = StreamController<String>.broadcast();
  String _rxBuffer = '';

  bool get isConnected => _sock != null;

  Stream<String> get lines => _linesCtrl.stream;

  Future<void> connect(String ip, int port,
      {Duration timeout = const Duration(seconds: 5)}) async {
    if (_sock != null) return;
    final sock = await Socket.connect(ip, port, timeout: timeout);
    _sock = sock;

    sock.listen((Uint8List data) {
      final text = utf8.decode(data, allowMalformed: true);
      final combined = _rxBuffer + text;
      final parts = combined.split('\n');

      // emit complete lines (strip CR)
      for (int i = 0; i < parts.length - 1; i++) {
        _linesCtrl.add(parts[i].replaceAll('\r', ''));
      }
      _rxBuffer = parts.last;
    }, onDone: () {
      // flush any tail
      if (_rxBuffer.isNotEmpty) {
        _linesCtrl.add(_rxBuffer.replaceAll('\r', ''));
        _rxBuffer = '';
      }
      disconnect();
      _linesCtrl.add('Disconnected (remote).');
    }, onError: (e) {
      _linesCtrl.add('Error: $e');
      disconnect();
    });
  }

  /// Send raw bytes (e.g., single byte 'A')
  void sendBytes(List<int> bytes) {
    _sock?.add(bytes);
  }

  /// Send a text line, ensures trailing '\n'
  void sendLine(String text) {
    final payload = text.endsWith('\n') ? text : '$text\n';
    _sock?.add(utf8.encode(payload));
  }

  void disconnect() {
    _sock?.destroy();
    _sock = null;
    _rxBuffer = '';
  }

  void dispose() {
    disconnect();
    _linesCtrl.close();
  }
}
