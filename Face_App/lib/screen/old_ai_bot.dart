import 'package:flutter/material.dart';
import 'package:google_generative_ai/google_generative_ai.dart';
import 'package:robit_display/screen/pudu_bot_face_animation.dart';
import 'package:robit_display/server/camera_feed_server.dart';
import 'package:speech_to_text/speech_to_text.dart' as stt;
import 'package:flutter_tts/flutter_tts.dart';

class ChatMessage {
  final String text;
  final bool isUser;
  ChatMessage({required this.text, required this.isUser});
}

class ChatScreens extends StatefulWidget {
  @override
  _ChatScreenState createState() => _ChatScreenState();
}

class _ChatScreenState extends State<ChatScreens> {
  final List<ChatMessage> _messages = [];
  final TextEditingController _textController = TextEditingController();
  bool _isLoading = false;
  bool _isListening = false;
  bool _isSpeaking = false;

  late GenerativeModel _model;
  late ChatSession _chat;
  late stt.SpeechToText _speech;
  late FlutterTts _flutterTts;

  @override
  void initState() {
    super.initState();
    _initializeChat();
    _initializeSpeech();
    _initializeTts();
    // Start listening when app opens
    Future.delayed(Duration(seconds: 1), () {
      _startListening();
    });
  }

  void _initializeChat() {
    final apiKey =
        'AIzaSyDKU0WRwt-S_ABs8i80_0Y-WHJFbS3BAw0'; // Get this from Google AI Studio
    _model = GenerativeModel(
      model: 'gemini-pro',
      apiKey: apiKey,
    );
    _chat = _model.startChat();
  }

  void _initializeSpeech() async {
    _speech = stt.SpeechToText();
    await _speech.initialize();
  }

  void _initializeTts() {
    _flutterTts = FlutterTts();
    _flutterTts.setLanguage('en-US');
    _flutterTts.setSpeechRate(0.5);

    // Listen for TTS completion
    _flutterTts.setCompletionHandler(() {
      setState(() => _isSpeaking = false);
      // Start listening after response is spoken
      _startListening();
    });

    // Set progress handler to track speaking state
    _flutterTts.setStartHandler(() {
      setState(() => _isSpeaking = true);
    });
  }

  Future<void> _speak(String text) async {
    if (_isListening) {
      await _stopListening();
    }
    await _flutterTts.speak(text);
  }

  Future<void> _startListening() async {
    if (!_isListening && !_isSpeaking) {
      bool available = await _speech.initialize();
      if (available) {
        setState(() => _isListening = true);
        try {
          await _speech.listen(
            onResult: (result) {
              if (result.finalResult) {
                setState(() => _isListening = false);
                if (result.recognizedWords.isNotEmpty) {
                  _handleSubmitted(result.recognizedWords);
                } else {
                  _startListening(); // Retry if no words recognized
                }
              }
            },
            listenMode: stt.ListenMode.confirmation,
            cancelOnError: true,
            partialResults: false,
          );
        } catch (e) {
          print('Error listening: $e');
          _startListening(); // Retry on error
        }
      }
    }
  }

  Future<void> _stopListening() async {
    await _speech.stop();
    setState(() => _isListening = false);
  }

  Future<void> _handleSubmitted(String text) async {
    if (text.isEmpty) return;

    _textController.clear();
    setState(() {
      _messages.add(ChatMessage(text: text, isUser: true));
      _isLoading = true;
    });

    try {
      final response = await _chat.sendMessage(Content.text(text));
      final responseText = response.text ?? 'No response';

      setState(() {
        _messages.add(ChatMessage(text: responseText, isUser: false));
        _isLoading = false;
      });

      // Speak the response - listening will auto-start after completion
      await _speak(responseText);
    } catch (e) {
      setState(() {
        _messages.add(ChatMessage(
          text: 'Error: Unable to get response',
          isUser: false,
        ));
        _isLoading = false;
      });
      _startListening(); // Restart listening on error
    }
  }

  Widget _buildStatusText() {
    if (_isSpeaking)
      return Text('Speaking...', style: TextStyle(color: Colors.blue));
    if (_isListening)
      return Text('Listening...', style: TextStyle(color: Colors.green));
    if (_isLoading)
      return Text('Processing...', style: TextStyle(color: Colors.orange));
    return Text('Tap mic to start', style: TextStyle(color: Colors.grey));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(children: [
        Column(
          children: [
            Expanded(
              child: ListView.builder(
                padding: EdgeInsets.all(8.0),
                itemCount: _messages.length,
                reverse: true, // Show latest messages at the bottom
                itemBuilder: (context, index) {
                  final message = _messages[_messages.length - 1 - index];
                  return Container(
                    margin: EdgeInsets.symmetric(vertical: 4.0),
                    alignment: message.isUser
                        ? Alignment.centerRight
                        : Alignment.centerLeft,
                    child: Container(
                      padding: EdgeInsets.symmetric(
                          horizontal: 16.0, vertical: 10.0),
                      decoration: BoxDecoration(
                        color: message.isUser
                            ? Colors.blue[400]
                            : Colors.grey[300],
                        borderRadius: BorderRadius.circular(20.0),
                      ),
                      constraints: BoxConstraints(maxWidth: 250.0),
                      child: Text(
                        message.text,
                        style: TextStyle(
                          color: message.isUser ? Colors.white : Colors.black,
                        ),
                      ),
                    ),
                  );
                },
              ),
            ),
            Container(
              decoration: BoxDecoration(
                border: Border(top: BorderSide(color: Colors.grey[300]!)),
              ),
              padding: EdgeInsets.symmetric(horizontal: 8.0),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  IconButton(
                    icon: Icon(
                      _isListening ? Icons.mic : Icons.mic_none,
                      size: 30,
                    ),
                    color: _isListening ? Colors.red : Colors.blue,
                    onPressed: _isListening ? _stopListening : _startListening,
                  ),
                ],
              ),
            ),
          ],
        ),
        GestureDetector(
          onTap: _isListening ? _stopListening : _startListening,
          child: Align(
            alignment: Alignment.center,
            child:
                CameraFeedServer(), // Ensure BellaBotFace always remains centered
          ),
        ),
      ]),
    );
  }

  @override
  void dispose() {
    _flutterTts.stop();
    _speech.stop();
    super.dispose();
  }
}
