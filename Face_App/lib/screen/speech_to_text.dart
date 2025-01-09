import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:robit_display/provider/voice_input.dart';

/// Widget that listens to provider state and displays the text and response
class SpeechToTextExample extends ConsumerWidget {
  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final speechState = ref.watch(speechToTextProvider);

    return Scaffold(
      appBar: AppBar(
        title: Text('Confidence: ${speechState.confidence * 100}%'),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              'Recognized Text: ${speechState.recognizedText}',
              style: TextStyle(fontSize: 20.0),
            ),
            SizedBox(height: 20),
            Text(
              'Server Response: ${speechState.serverResponse}',
              style: TextStyle(fontSize: 20.0),
            ),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          ref.read(speechToTextProvider.notifier).startListening();
        },
        child: Icon(Icons.mic),
      ),
    );
  }
}
