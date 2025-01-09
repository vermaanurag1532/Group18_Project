import 'dart:async';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:robit_display/main.dart';
import 'package:robit_display/provider/face_cordinate_provider.dart';

class BellaBotFace extends ConsumerStatefulWidget {
  @override
  _BellaBotFaceState createState() => _BellaBotFaceState();
}

class _BellaBotFaceState extends ConsumerState<BellaBotFace>
    with TickerProviderStateMixin {
  late AnimationController _blinkController;
  late Animation<double> _blinkAnimation;
  late Timer _blinkTimer;

  @override
  void initState() {
    super.initState();

    // Initialize the blink controller and animation
    _blinkController = AnimationController(
      duration: Duration(milliseconds: 150),
      vsync: this,
    );
    _blinkAnimation = Tween<double>(begin: 1.0, end: 0.0).animate(
      CurvedAnimation(parent: _blinkController, curve: Curves.easeInOut),
    );

    // Start the blink timer
    _blinkTimer = Timer.periodic(Duration(seconds: 5), (timer) {
      _blinkController.forward().then((_) => _blinkController.reverse());
    });
  }

  @override
  void dispose() {
    _blinkController.dispose();
    _blinkTimer.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final faceCoordinates = ref.watch(faceCoordinatesProvider);

    // Default to center if no face is detected
    Offset offset = Offset.zero;
    double pointerX = 0.0;
    double pointerY = 0.0;

    // Set pointer offsets based on detected face coordinates (if available)
    // If face coordinates are available, calculate the offset
    if (faceCoordinates.isNotEmpty) {
      final face = faceCoordinates.first;
      const maxEyeBallDistance = 44; // Max pupil movement within the eye

      // Normalize coordinates for eye movement
      final percentPositionX = maxEyeBallDistance / screenWidth;
      final percentPositionY = maxEyeBallDistance / screenHeight;

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

    return Scaffold(
      backgroundColor: Colors.black,
      body: Stack(
        alignment: Alignment.center,
        children: [
          SizedBox(
            height: screenWidth / 30,
            width: screenWidth / 2,
            child: CustomPaint(
              size: Size(screenWidth, screenHeight), // Adjust size as needed
              painter: LeftEyeBrow(),
            ),
          ),
          SizedBox(
            height: screenWidth / 30,
            width: screenWidth / 2,
            child: CustomPaint(
              size: Size(screenWidth, screenHeight), // Adjust size as needed
              painter: RightEyeBrow(),
            ),
          ),

          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              AnimatedBuilder(
                animation: _blinkController,
                builder: (context, child) {
                  return BellaBotEye(
                    animation: _blinkAnimation,
                    offset: Offset(pointerX, pointerY),
                  );
                },
              ),
              SizedBox(width: 120),
              AnimatedBuilder(
                animation: _blinkController,
                builder: (context, child) {
                  return BellaBotEye(
                    animation: _blinkAnimation,
                    offset: Offset(pointerX, pointerY),
                  );
                },
              ),
            ],
          ),
          // Other face elements like nose and mouth...
          // Nose and Mouth
          Column(
            mainAxisAlignment: MainAxisAlignment.end,
            children: [
              Container(
                width: screenWidth / 35,
                height: screenWidth / 40,
                decoration: BoxDecoration(
                  color: Colors.white,
                  borderRadius: BorderRadius.only(
                    topLeft: Radius.circular(12),
                    topRight: Radius.circular(12),
                    bottomLeft: Radius.circular(20),
                    bottomRight: Radius.circular(20),
                  ),
                ),
              ),
              SizedBox(
                height: screenHeight / 50,
              ),

              // Mouth (cat-like)
              SizedBox(
                height: screenWidth / 30,
                width: screenWidth / 2,
                child: CustomPaint(
                  size:
                      Size(screenWidth, screenHeight), // Adjust size as needed
                  painter: WShapePainter(),
                ),
              ),
              SizedBox(
                height: screenHeight / 5.2,
              )
            ],
          ),
        ],
      ),
    );
  }
}

class BellaBotEye extends StatelessWidget {
  final Animation<double> animation;
  final Offset offset;

  BellaBotEye({required this.animation, required this.offset});

  @override
  Widget build(BuildContext context) {
    // Dimensions for the open eye and blink shape
    final double eyeDiameter = (3 * screenWidth) / 10;
    final double closedHeight = eyeDiameter / 7; // Height of closed eye

    return AnimatedBuilder(
      animation: animation,
      builder: (context, child) {
        // Eye height transitions from full diameter to a thin rounded rectangle
        double currentHeight = eyeDiameter * animation.value +
            closedHeight * (1 - animation.value);

        return ClipRRect(
          borderRadius: BorderRadius.circular(
              animation.value < 0.1 ? 15 : eyeDiameter / 2),
          child: Container(
            width: eyeDiameter,
            height: currentHeight,
            decoration: BoxDecoration(
              color: Colors.white,
              borderRadius: BorderRadius.circular(
                  animation.value < 0.1 ? 15 : eyeDiameter / 2),
            ),
            child: Transform.translate(
              offset: offset,
              child: Center(
                child: ClipOval(
                  child: Container(
                    width: eyeDiameter / 1.5, // Fixed width for the pupil
                    height: eyeDiameter / 1.5 * animation.value,
                    color: Colors.black,
                  ),
                ),
              ),
            ),
          ),
        );
      },
    );
  }
}

class WShapePainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.white
      ..strokeWidth = 12.0
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    final path = Path();
    final double wHeight = size.height;
    final double wWidth = size.width;

    // Start point (top left)
    path.moveTo(wWidth * 0.40, wHeight * 0.10);

    // Create 'W' with smooth curves at each turn

    path.arcToPoint(
      Offset(wWidth * 0.5, wHeight * 0.1),
      radius: Radius.circular(10),
      clockwise: false,
    );
    path.arcToPoint(
      Offset(wWidth * 0.6, wHeight * 0.1),
      radius: Radius.circular(10),
      clockwise: false,
    );

    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}

class LeftEyeBrow extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.white
      ..strokeWidth = 30.0
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    final path = Path();
    final double wHeight = size.height;
    final double wWidth = size.width;

    // Start point (top left)
    path.moveTo(wWidth * 0.30, -wHeight * 5);
    path.lineTo(wWidth * 0.37, -wHeight * 5);

    // Create 'W' with smooth curves at each turn

    // path.arcToPoint(
    //   Offset(wWidth * 0.5, wHeight * 0.1),
    //   radius: Radius.circular(10),
    //   clockwise: false,
    // );

    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}

class RightEyeBrow extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.white
      ..strokeWidth = 30.0
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;

    final path = Path();
    final double wHeight = size.height;
    final double wWidth = size.width;

    // Start point (top left)
    path.moveTo(wWidth * 0.63, -wHeight * 5);
    path.lineTo(wWidth * 0.7, -wHeight * 5);

    // Create 'W' with smooth curves at each turn

    // path.arcToPoint(
    //   Offset(wWidth * 0.5, wHeight * 0.1),
    //   radius: Radius.circular(10),
    //   clockwise: false,
    // );

    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}
