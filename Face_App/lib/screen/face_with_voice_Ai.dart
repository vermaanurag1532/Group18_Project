// import 'package:flutter/material.dart';
// import 'package:robit_display/screen/old_ai_bot.dart';
// import 'package:speech_to_text/speech_to_text.dart';

// class ChatScreenWithFace extends StatefulWidget {
//   @override
//   _ChatScreenWithFaceState createState() => _ChatScreenWithFaceState();
// }

// class _ChatScreenWithFaceState extends State<ChatScreenWithFace> {
//   // Existing voice logic
//   final List<ChatMessage> _messages = [];
//   final TextEditingController _textController = TextEditingController();
//   bool _isLoading = false;
//   bool _isListening = false;
//   bool _isSpeaking = false;

//   late stt.SpeechToText _speech;
//   late FlutterTts _flutterTts;
//   late GenerativeModel _model;
//   late ChatSession _chat;

//   @override
//   void initState() {
//     super.initState();
//     _initializeChat();
//     _initializeSpeech();
//     _initializeTts();

//     // Start listening automatically
//     Future.delayed(Duration(seconds: 1), () {
//       _startListening();
//     });
//   }

//   // Initialization methods...

//   Future<void> _speak(String text) async {
//     if (_isListening) await _stopListening();
//     await _flutterTts.speak(text);
//   }

//   Future<void> _startListening() async {
//     if (!_isListening && !_isSpeaking) {
//       bool available = await _speech.initialize();
//       if (available) {
//         setState(() => _isListening = true);
//         await _speech.listen(
//           onResult: (result) {
//             if (result.finalResult) {
//               setState(() => _isListening = false);
//               _handleSubmitted(result.recognizedWords);
//             }
//           },
//           listenMode: stt.ListenMode.confirmation,
//           partialResults: false,
//         );
//       }
//     }
//   }

//   Future<void> _handleSubmitted(String text) async {
//     setState(() {
//       _messages.add(ChatMessage(text: text, isUser: true));
//       _isLoading = true;
//     });

//     try {
//       final response = await _chat.sendMessage(Content.text(text));
//       final responseText = response.text ?? 'No response';
//       setState(() {
//         _messages.add(ChatMessage(text: responseText, isUser: false));
//         _isLoading = false;
//       });

//       await _speak(responseText);
//     } catch (e) {
//       setState(() {
//         _messages.add(ChatMessage(text: 'Error: Unable to get response', isUser: false));
//         _isLoading = false;
//       });
//     }
//   }

//   @override
//   Widget build(BuildContext context) {
//     return Scaffold(
//       body: Stack(
//         children: [
//           // Chat UI
//           Column(
//             children: [
//               Expanded(
//                 child: ListView.builder(
//                   padding: EdgeInsets.all(8.0),
//                   itemCount: _messages.length,
//                   reverse: true,
//                   itemBuilder: (context, index) {
//                     final message = _messages[_messages.length - 1 - index];
//                     return Container(
//                       margin: EdgeInsets.symmetric(vertical: 4.0),
//                       alignment: message.isUser
//                           ? Alignment.centerRight
//                           : Alignment.centerLeft,
//                       child: Container(
//                         padding: EdgeInsets.symmetric(horizontal: 16.0, vertical: 10.0),
//                         decoration: BoxDecoration(
//                           color: message.isUser ? Colors.blue[400] : Colors.grey[300],
//                           borderRadius: BorderRadius.circular(20.0),
//                         ),
//                         constraints: BoxConstraints(maxWidth: 250.0),
//                         child: Text(
//                           message.text,
//                           style: TextStyle(
//                             color: message.isUser ? Colors.white : Colors.black,
//                           ),
//                         ),
//                       ),
//                     );
//                   },
//                 ),
//               ),
//               // Microphone Button
//               Container(
//                 decoration: BoxDecoration(
//                   border: Border(top: BorderSide(color: Colors.grey[300]!)),
//                 ),
//                 padding: EdgeInsets.symmetric(horizontal: 8.0),
//                 child: Row(
//                   mainAxisAlignment: MainAxisAlignment.center,
//                   children: [
//                     IconButton(
//                       icon: Icon(
//                         _isListening ? Icons.mic : Icons.mic_none,
//                         size: 30,
//                       ),
//                       color: _isListening ? Colors.red : Colors.blue,
//                       onPressed: _isListening ? _stopListening : _startListening,
//                     ),
//                   ],
//                 ),
//               ),
//             ],
//           ),
//           // Overlay BellaBotFace UI
//           Align(
//             alignment: Alignment.center,
//             child: BellaBotFace(), // Ensure BellaBotFace always remains centered
//           ),
//         ],
//       ),
//     );
//   }

//   @override
//   void dispose() {
//     _flutterTts.stop();
//     _speech.stop();
//     super.dispose();
//   }
// }
