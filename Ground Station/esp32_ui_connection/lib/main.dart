import 'package:flutter/material.dart';
import 'screens/drone_tcp_console.dart';
// main.dart is the app entry point. Its purpose is to:
//  1. start the Flutter application
//  2. set global MaterialApp option (Flutter jargon)
//  3. Chooses the screen to load first: DroneTcpConsole()
void main() => runApp(const MaterialApp(
      debugShowCheckedModeBanner: false,
      home: DroneTcpConsole(),
    ));
