import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';

import 'package:robit_display/screen/speech_to_text.dart';

class FaceRegistrationScreen extends StatefulWidget {
  @override
  _FaceRegistrationScreenState createState() => _FaceRegistrationScreenState();
}

class _FaceRegistrationScreenState extends State<FaceRegistrationScreen> {
  final TextEditingController _nameController = TextEditingController();
  bool _isSubmitting = false;
  String? _message;

  Future<void> _submitName() async {
    final name = _nameController.text.trim();
    if (name.isEmpty) {
      setState(() {
        _message = "Please enter a name.";
      });
      return;
    }

    setState(() {
      _isSubmitting = true;
      _message = null;
    });

    try {
      final response = await http.post(
        Uri.parse(
            'http://192.168.29.36:6000/submit_name'), // Replace <YOUR_SERVER_IP>
        headers: {"Content-Type": "application/json"},
        body: json.encode({"name": name}),
      );

      if (response.statusCode == 200) {
        setState(() {
          _message = "Name submitted successfully!";
        });
        //Navigator.of(context).pop();
      } else {
        setState(() {
          _message = "Error submitting name: ${response.body}";
        });
        //Navigator.of(context).pop();
      }
    } catch (error) {
      setState(() {
        _message = "Failed to connect to server.";
      });
      //Navigator.of(context).pop();
    } finally {
      setState(() {
        _isSubmitting = false;
        _nameController.clear();
      });
      Navigator.of(context).pop();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text("Face Registration")),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            //SpeechToTextExample(),
            TextField(
              controller: _nameController,
              decoration: InputDecoration(
                labelText: "Enter name",
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 16),
            ElevatedButton(
              onPressed: _isSubmitting ? null : _submitName,
              child: _isSubmitting
                  ? CircularProgressIndicator()
                  : Text("Submit Name"),
            ),
            SizedBox(height: 16),
            if (_message != null)
              Text(
                _message!,
                style: TextStyle(color: Colors.blueAccent),
              ),
          ],
        ),
      ),
    );
  }
}
