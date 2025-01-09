import 'dart:math';
import 'dart:developer' as console;
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:robit_display/provider/face_cordinate_provider.dart';

class Smiley extends ConsumerStatefulWidget {
  const Smiley({super.key});

  @override
  SmileyState createState() => SmileyState();
}

class SmileyState extends ConsumerState<Smiley> {
  @override
  Widget build(BuildContext context) {
    final faceCoordinates = ref.watch(faceCoordinatesProvider);

    // Default to center if no face is detected
    Offset offset = Offset.zero;
    double pointerX = 0.0;
    double pointerY = 0.0;

    console.log("New Cordinates");

    if (faceCoordinates.isNotEmpty) {
      final face = faceCoordinates.first;
      console.log("x = ${face.x}");

      // Get screen size and define eye movement limits
      final screenWidth = MediaQuery.of(context).size.width;
      final screenHeight = MediaQuery.of(context).size.height;
      const maxEyeBallDistance = 44;

      final mediaX = MediaQuery.of(context).size.width;
      final mediaY = MediaQuery.of(context).size.height;

      final percentPositionX = maxEyeBallDistance / mediaX;
      final percentPositionY = maxEyeBallDistance / mediaY;

      setState(() {
        pointerX =
            percentPositionX * (screenWidth - face.x) - maxEyeBallDistance / 2;
        pointerY = percentPositionY * face.y - maxEyeBallDistance / 2;
      });

      // Normalize coordinates relative to screen dimensions
      final normalizedX = (face.x / screenWidth) * maxEyeBallDistance;
      final normalizedY = (face.y / screenHeight) * maxEyeBallDistance;

      offset = Offset(
        normalizedX - maxEyeBallDistance / 2,
        normalizedY - maxEyeBallDistance / 2,
      );

      // Cap offset to keep the eyeball within range
      final maxPointerDistance = maxEyeBallDistance / 2;
      final pointerDistance =
          sqrt(offset.dx * offset.dx + offset.dy * offset.dy);

      if (pointerDistance > maxPointerDistance) {
        final angle = atan2(offset.dy, offset.dx);
        offset = Offset(
          cos(angle) * maxPointerDistance,
          sin(angle) * maxPointerDistance,
        );
      }
    }

    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Stack(
            alignment: Alignment.center,
            children: [
              CustomPaint(
                painter: Face(),
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  EyeBall(offset: Offset(pointerX, pointerY)),
                  const SizedBox(width: 60),
                  EyeBall(offset: Offset(pointerX, pointerY)),
                ],
              ),
            ],
          ),
        ],
      ),
    );
  }
}

class Face extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final facePaint = Paint()
      ..color = Colors.yellowAccent
      ..style = PaintingStyle.fill;
    canvas.drawCircle(center.translate(0, 30), 150, facePaint);

    final nosePaint = Paint()
      ..style = PaintingStyle.fill
      ..color = Colors.black;
    canvas.drawCircle(center.translate(0.0, 60), 15, nosePaint);

    final smilePaint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..color = Colors.redAccent[700]!
      ..strokeWidth = 10;
    canvas.drawRect(
      Rect.fromCenter(center: center.translate(0.0, 100), width: 60, height: 5),
      smilePaint,
    );
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

class EyeBall extends StatelessWidget {
  final Offset offset;

  const EyeBall({Key? key, required this.offset}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Container(
      height: 60,
      width: 60,
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        color: Colors.white,
        border: Border.all(color: Colors.black),
      ),
      child: Transform.translate(
        offset: offset,
        child: CustomPaint(painter: Pointer(7)),
      ),
    );
  }
}

class Pointer extends CustomPainter {
  final double radius;

  Pointer(this.radius);

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final paint = Paint()
      ..color = Colors.white
      ..blendMode = BlendMode.difference;
    canvas.drawCircle(center, radius, paint);
    canvas.drawCircle(center.translate(0.0, 4.0), 2, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}
