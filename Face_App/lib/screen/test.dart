import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter_tts/flutter_tts.dart';
import 'package:speech_to_text/speech_to_text.dart' as stt;
import 'package:http/http.dart' as http;
import 'package:image_picker/image_picker.dart';

class FaceRecognitionApp extends StatefulWidget {
  @override
  _FaceRecognitionAppState createState() => _FaceRecognitionAppState();
}

class _FaceRecognitionAppState extends State<FaceRecognitionApp> {
  final FlutterTts flutterTts = FlutterTts();
  final stt.SpeechToText speechToText = stt.SpeechToText();
  final picker = ImagePicker();
  bool isRecognizing = false;

  @override
  void initState() {
    super.initState();
    initSpeechRecognizer();
  }

  void initSpeechRecognizer() async {
    await speechToText.initialize();
  }

  Future<void> recognizeFace() async {
    setState(() {
      isRecognizing = true;
    });

    final pickedImage = await picker.pickImage(source: ImageSource.camera);
    if (pickedImage == null) {
      await flutterTts.speak("No image captured.");
      setState(() {
        isRecognizing = false;
      });
      return;
    }

    final imageBytes = await pickedImage.readAsBytes();
    final response = await http.post(
      Uri.parse('http://192.168.1.44:6000/detect_and_recognize'),
      headers: {"Content-Type": "application/json"},
      body: imageBytes,
    );

    if (response.statusCode == 200) {
      List<dynamic> faceData = jsonDecode(response.body);
      if (faceData.isNotEmpty && faceData[0]["name"] == "Unknown") {
        await flutterTts
            .speak("Face not recognized. Please tell me your name.");
        String userName = await takeVoiceInput();

        if (userName.isNotEmpty) {
          await registerFace(imageBytes, userName);
        }
      } else if (faceData.isNotEmpty) {
        await flutterTts.speak("Hello, ${faceData[0]["name"]}");
      } else {
        await flutterTts.speak("No face detected.");
      }
    } else {
      await flutterTts.speak("Error in face detection.");
    }

    setState(() {
      isRecognizing = false;
    });
  }

  Future<String> takeVoiceInput() async {
    bool isAvailable = await speechToText.initialize();
    if (isAvailable) {
      await speechToText.listen();
      await Future.delayed(Duration(seconds: 3));
      await speechToText.stop();
      if (speechToText.lastRecognizedWords.isNotEmpty) {
        return speechToText.lastRecognizedWords;
      }
    }
    return "";
  }

  Future<void> registerFace(List<int> imageBytes, String name) async {
    var request = http.MultipartRequest(
      'POST',
      Uri.parse('http://192.168.1.44:6000/register'),
    );
    request.fields['name'] = name;
    request.files.add(http.MultipartFile.fromBytes(
      'image',
      imageBytes,
      filename: 'face.jpg',
    ));

    var response = await request.send();
    if (response.statusCode == 200) {
      await flutterTts
          .speak("Thank you, $name. Your face has been registered.");
    } else {
      await flutterTts.speak("Failed to register face. Please try again.");
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("Face Recognition"),
      ),
      body: Center(
        child: isRecognizing
            ? CircularProgressIndicator()
            : ElevatedButton(
                onPressed: recognizeFace,
                child: Text("Detect Face"),
              ),
      ),
    );
  }
}
