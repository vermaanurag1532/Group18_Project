import 'dart:developer';

import 'package:flutter/material.dart';
import 'package:google_generative_ai/google_generative_ai.dart';
import 'package:speech_to_text/speech_to_text.dart' as stt;
import 'package:flutter_tts/flutter_tts.dart';
import 'package:translator/translator.dart';

enum Language { english, hindi }

class ChatMessage {
  final String text;
  final bool isUser;
  final Language language;
  ChatMessage(
      {required this.text,
      required this.isUser,
      this.language = Language.english});
}

class ChatScreen extends StatefulWidget {
  @override
  _ChatScreenState createState() => _ChatScreenState();
}

class _ChatScreenState extends State<ChatScreen> {
  final List<ChatMessage> _messages = [];
  final translator = GoogleTranslator();
  bool _isLoading = false;
  bool _isListening = false;
  bool _isSpeaking = false;
  Language _currentLanguage = Language.english;

  // Voice settings
  double _pitch = 1.0;
  double _rate = 0.5;
  double _volume = 1.0;

  late GenerativeModel _model;
  late ChatSession _chat;
  late stt.SpeechToText _speech;
  late FlutterTts _flutterTts;

  bool _isWakeWordMode = true;
  final String _wakeWord = "hey robot";

  @override
  void initState() {
    super.initState();
    _initializeChat();
    _initializeSpeech();
    _initializeTts();
    // Start listening for wake word
    Future.delayed(Duration(seconds: 1), () {
      _startWakeWordListening();
    });
  }

  void _initializeChat() {
    final apiKey =
        'AIzaSyDKU0WRwt-S_ABs8i80_0Y-WHJFbS3BAw0'; // Get from Google AI Studio
    _model = GenerativeModel(
      model: 'gemini-pro',
      apiKey: apiKey,
    );
    _chat = _model.startChat();
  }

  void _initializeSpeech() async {
    _speech = stt.SpeechToText();
    await _speech.initialize(
      onStatus: (status) {
        if (status == 'done') {
          if (_isWakeWordMode) {
            Future.delayed(Duration(milliseconds: 500), () {
              _startWakeWordListening();
            });
          }
        }
      },
    );
  }

  void _initializeTts() {
    _flutterTts = FlutterTts();
    _updateTtsSettings();

    _flutterTts.setCompletionHandler(() {
      setState(() => _isSpeaking = false);
      // Start listening for wake word again after speaking
      _startWakeWordListening();
    });
  }

  void _updateTtsSettings() async {
    await _flutterTts.setPitch(_pitch);
    await _flutterTts.setSpeechRate(_rate);
    await _flutterTts.setVolume(_volume);
    await _flutterTts
        .setLanguage(_currentLanguage == Language.english ? 'en-US' : 'hi-IN');
  }

  Future<void> _speak(String text) async {
    if (_isListening) {
      await _speech.stop();
      setState(() => _isListening = false);
    }

    if (_currentLanguage == Language.hindi) {
      final translation = await translator.translate(text, to: 'hi');
      text = translation.text;
    }

    setState(() => _isSpeaking = true);
    await _flutterTts.speak(text);
  }

  void _startCommandListening() async {
    if (!_isListening && !_isSpeaking) {
      setState(() {
        _isListening = true;
        _isWakeWordMode = false;
      });

      try {
        bool available = await _speech.initialize(
          onStatus: (status) {
            log("Command status: $status");
          },
        );

        if (available) {
          // Add a short delay before starting to listen for command
          await Future.delayed(Duration(milliseconds: 500));

          await _speech.listen(
            onResult: (result) {
              print("Command result: ${result.recognizedWords}");
              if (result.finalResult && result.recognizedWords.isNotEmpty) {
                _handleSubmitted(result.recognizedWords);
              }
            },
            listenFor: Duration(seconds: 10), // Set maximum listening time
            pauseFor: Duration(seconds: 3), // Set pause threshold
            cancelOnError: true,
            partialResults: false,
            localeId: _currentLanguage == Language.english ? 'en_US' : 'hi_IN',
          );
        }
      } catch (e) {
        print("Error in command listening: $e");
        setState(() => _isListening = false);
        _startWakeWordListening();
      }
    }
  }

  void _startWakeWordListening() async {
    if (!_isListening && !_isSpeaking) {
      setState(() {
        _isWakeWordMode = true;
        _isListening = true;
      });

      try {
        bool available = await _speech.initialize(
          onStatus: (status) {
            log("Wake word status: $status");
          },
        );

        if (available) {
          await _speech.listen(
            onResult: (result) {
              print("Wake word result: ${result.recognizedWords}");
              if (result.finalResult) {
                String text = result.recognizedWords.toLowerCase();
                if (text.contains(_wakeWord)) {
                  _speech.stop();
                  setState(() {
                    _isWakeWordMode = false;
                    _isListening = false;
                  });

                  // Add delay before speaking response
                  Future.delayed(Duration(milliseconds: 500), () {
                    _speak("Yes, I'm listening").then((_) {
                      // Start command listening after response
                      Future.delayed(Duration(milliseconds: 1500), () {
                        _startCommandListening();
                      });
                    });
                  });
                } else {
                  // If not wake word, restart listening
                  _startWakeWordListening();
                }
              }
            },
            cancelOnError: true,
            listenFor: Duration(seconds: 5),
            pauseFor: Duration(seconds: 2),
            partialResults: false,
            localeId: _currentLanguage == Language.english ? 'en_US' : 'hi_IN',
          );
        }
      } catch (e) {
        print("Error in wake word listening: $e");
        Future.delayed(Duration(milliseconds: 500), () {
          _startWakeWordListening();
        });
      }
    }
  }

  Future<void> _handleSubmitted(String text) async {
    print("Processing command: $text");
    if (text.isEmpty) return;

    // Stop listening while processing
    if (_isListening) {
      await _speech.stop();
    }

    setState(() {
      _messages.add(
          ChatMessage(text: text, isUser: true, language: _currentLanguage));
      _isLoading = true;
      _isListening = false;
    });

    try {
      String processedText = text;
      if (_currentLanguage == Language.hindi) {
        final translation = await translator.translate(text, to: 'en');
        processedText = translation.text;
      }

      final response = await _chat.sendMessage(Content.text(processedText));
      final responseText = response.text ?? 'No response';

      setState(() {
        _messages.add(ChatMessage(
            text: responseText, isUser: false, language: _currentLanguage));
        _isLoading = false;
      });

      // Add delay before speaking response
      await Future.delayed(Duration(milliseconds: 500));
      await _speak(responseText);

      // Return to wake word mode after response
      Future.delayed(Duration(milliseconds: 1000), () {
        setState(() => _isWakeWordMode = true);
        _startWakeWordListening();
      });
    } catch (e) {
      print("Error processing command: $e");
      setState(() {
        _messages.add(ChatMessage(
            text: 'Error: Unable to get response',
            isUser: false,
            language: _currentLanguage));
        _isLoading = false;
      });

      Future.delayed(Duration(milliseconds: 1000), () {
        setState(() => _isWakeWordMode = true);
        _startWakeWordListening();
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Voice Assistant'),
        actions: [
          IconButton(
            icon: Text(_currentLanguage == Language.english ? 'EN' : 'हि'),
            onPressed: () {
              setState(() {
                _currentLanguage = _currentLanguage == Language.english
                    ? Language.hindi
                    : Language.english;
              });
              _updateTtsSettings();
            },
          ),
          IconButton(
            icon: Icon(Icons.settings),
            onPressed: () {
              showModalBottomSheet(
                context: context,
                builder: (context) => _buildVoiceControls(),
              );
            },
          ),
        ],
      ),
      body: Column(
        children: [
          Container(
            padding: EdgeInsets.all(8),
            color: _isWakeWordMode
                ? Colors.blue.withOpacity(0.1)
                : Colors.green.withOpacity(0.1),
            child: Text(
              _isWakeWordMode
                  ? 'Say "Hey Robot" to wake me up'
                  : 'Listening for command...',
              style: TextStyle(color: Colors.grey[700]),
            ),
          ),
          Expanded(
            child: ListView.builder(
              padding: EdgeInsets.all(8.0),
              itemCount: _messages.length,
              reverse: true,
              itemBuilder: (context, index) {
                final message = _messages[_messages.length - 1 - index];
                return Container(
                  margin: EdgeInsets.symmetric(vertical: 4.0),
                  alignment: message.isUser
                      ? Alignment.centerRight
                      : Alignment.centerLeft,
                  child: Container(
                    padding:
                        EdgeInsets.symmetric(horizontal: 16.0, vertical: 10.0),
                    decoration: BoxDecoration(
                      color:
                          message.isUser ? Colors.blue[400] : Colors.grey[300],
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
          if (_isLoading || _isListening || _isSpeaking)
            Container(
              padding: EdgeInsets.all(8),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  if (_isLoading) CircularProgressIndicator(),
                  if (_isListening) Icon(Icons.mic, color: Colors.red),
                  if (_isSpeaking) Icon(Icons.volume_up, color: Colors.blue),
                ],
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildVoiceControls() {
    return Container(
      padding: EdgeInsets.all(16),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Text('Voice Settings',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
          SizedBox(height: 10),
          Row(
            children: [
              Text('Pitch: '),
              Expanded(
                child: Slider(
                  value: _pitch,
                  min: 0.5,
                  max: 2.0,
                  onChanged: (value) {
                    setState(() => _pitch = value);
                    _updateTtsSettings();
                  },
                ),
              ),
            ],
          ),
          Row(
            children: [
              Text('Speed: '),
              Expanded(
                child: Slider(
                  value: _rate,
                  min: 0.1,
                  max: 1.0,
                  onChanged: (value) {
                    setState(() => _rate = value);
                    _updateTtsSettings();
                  },
                ),
              ),
            ],
          ),
          Row(
            children: [
              Text('Volume: '),
              Expanded(
                child: Slider(
                  value: _volume,
                  min: 0.0,
                  max: 1.0,
                  onChanged: (value) {
                    setState(() => _volume = value);
                    _updateTtsSettings();
                  },
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    _flutterTts.stop();
    _speech.stop();
    super.dispose();
  }
}
