import 'dart:math';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:robit_display/data/model/face_ccordinate_model.dart';
import 'package:robit_display/provider/face_cordinate_provider.dart';

class EyeAnimationScreen extends ConsumerStatefulWidget {
  const EyeAnimationScreen({super.key});

  @override
  ConsumerState<EyeAnimationScreen> createState() => _EyeAnimationScreenState();
}

class _EyeAnimationScreenState extends ConsumerState<EyeAnimationScreen> {
  Offset pupilOffset = Offset.zero;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _updatePupilPosition();
  }

  // Update the pupil offset based on the nearest face
  void _updatePupilPosition() {
    final faceData = ref.watch(faceCoordinatesProvider);
    if (faceData.isEmpty) return;

    // Screen center point for "head"
    const Offset screenCenter = Offset(0.5, 0.5);
    FaceCoordinate nearestFace = faceData[0];
    double nearestDistance = double.infinity;

    for (var face in faceData) {
      final faceCenter =
          Offset(face.x + face.width / 2, face.y + face.height / 2);
      final distance = (screenCenter - faceCenter).distance;
      if (distance < nearestDistance) {
        nearestFace = face;
        nearestDistance = distance;
      }
    }

    // Map face coordinates to pupil range and set pupil offset
    final Offset targetOffset = Offset(
      (nearestFace.x + nearestFace.width / 2 - 0.5) *
          2, // Adjust scale to center pupil
      (nearestFace.y + nearestFace.height / 2 - 0.5) * 2,
    );

    setState(() {
      pupilOffset = targetOffset;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.grey[900],
      body: Center(
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            EyeWidget(pupilOffset: pupilOffset), // Left eye
            const SizedBox(width: 20),
            EyeWidget(pupilOffset: pupilOffset), // Right eye
          ],
        ),
      ),
    );
  }
}

class EyeWidget extends StatelessWidget {
  final Offset pupilOffset;

  const EyeWidget({Key? key, required this.pupilOffset}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: 100,
      height: 100,
      child: CustomPaint(
        painter: EyePainter(pupilOffset),
      ),
    );
  }
}

class EyePainter extends CustomPainter {
  final Offset pupilOffset;

  EyePainter(this.pupilOffset);

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = min(size.width, size.height) / 2;

    // Draw eyeball
    final paint = Paint()..color = Colors.white;
    canvas.drawCircle(center, radius, paint);

    // Draw pupil, centered within eyeball bounds
    final pupilRadius = radius / 3;
    final pupilCenter = center +
        Offset(pupilOffset.dx * pupilRadius, pupilOffset.dy * pupilRadius);
    paint.color = Colors.black;
    canvas.drawCircle(pupilCenter, pupilRadius, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
