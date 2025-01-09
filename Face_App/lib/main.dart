import 'package:flutter/material.dart';
import 'package:camera/camera.dart';
import 'package:robit_display/screen/ai_chatbot.dart';
import 'package:robit_display/screen/face_with_voice_Ai.dart';

import 'package:robit_display/screen/pudu_bot_face_animation.dart';
import 'package:robit_display/screen/speech_to_text.dart';
import 'package:robit_display/server/camera_feed_server.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:flutter/services.dart';

import 'screen/old_ai_bot.dart';

late List<CameraDescription> cameras;
late double screenHeight;
late double screenWidth;

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  cameras = await availableCameras();

  // Set to full-screen mode and hide status and navigation bars
  SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);

  runApp(const ProviderScope(child: MyApp()));
}

class MyApp extends StatefulWidget {
  const MyApp({super.key});

  @override
  State<MyApp> createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  @override
  Widget build(BuildContext context) {
    screenWidth = MediaQuery.of(context).size.width;
    screenHeight = MediaQuery.of(context).size.height;
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: ChatScreens(),
    );
  }
}
