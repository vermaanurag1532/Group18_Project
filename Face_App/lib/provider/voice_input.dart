import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:speech_to_text/speech_to_text.dart' as stt;
import 'package:flutter_tts/flutter_tts.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';

/// State class to store recognized text and server response
class SpeechToTextState {
  final String recognizedText;
  final String serverResponse;
  final double confidence;

  SpeechToTextState({
    required this.recognizedText,
    required this.serverResponse,
    required this.confidence,
  });
}

/// Provider that handles speech recognition and server communication
class SpeechToTextNotifier extends Notifier<SpeechToTextState> {
  late stt.SpeechToText _speech;
  late FlutterTts _flutterTts;

  /// The build method initializes the state for the provider.
  @override
  SpeechToTextState build() {
    _speech = stt.SpeechToText();
    _flutterTts = FlutterTts();
    return SpeechToTextState(
        recognizedText: '', serverResponse: '', confidence: 1.0);
  }

  /// Initialize and start listening for voice input
  Future<void> startListening() async {
    bool available = await _speech.initialize(
      onStatus: (val) => print("Listening status: $val"),
      onError: (val) => print('Error: $val'),
    );

    if (available) {
      _speech.listen(
        onResult: (val) {
          if (val.confidence > 0.7) {
            final recognizedText = val.recognizedWords;
            final confidence = val.confidence;
            _sendToServer(recognizedText, confidence);
          }
        },
        listenFor: Duration(minutes: 1),
        pauseFor: Duration(seconds: 5),
        listenOptions: stt.SpeechListenOptions(
          partialResults: true,
          listenMode: stt.ListenMode.dictation,
        ),
      );
    }
  }

  /// Send recognized text to the server and get the response
  Future<void> _sendToServer(String text, double confidence) async {
    final url = Uri.parse('http://192.168.29.235:3000/extract-name');
    try {
      final response = await http.post(
        url,
        headers: {"Content-Type": "application/json"},
        body: jsonEncode({"text": text}),
      );

      if (response.statusCode == 200) {
        final serverResponse = jsonDecode(response.body)['response'];
        state = SpeechToTextState(
          recognizedText: text,
          serverResponse: serverResponse,
          confidence: confidence,
        );
        _speak(serverResponse);
      } else {
        print('Server error: ${response.statusCode}');
      }
    } catch (e) {
      print('Failed to connect to the server: $e');
    }
  }

  /// Text-to-speech function to read out the server response
  Future<void> _speak(String text) async {
    await _flutterTts.speak(text);
  }

  /// Stop listening
  void stopListening() {
    _speech.stop();
  }
}

/// Riverpod provider instance
final speechToTextProvider =
    NotifierProvider<SpeechToTextNotifier, SpeechToTextState>(
  () => SpeechToTextNotifier(),
);
