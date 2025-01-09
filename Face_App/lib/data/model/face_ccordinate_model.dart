class FaceCoordinate {
  final double x;
  final double y;
  final double width;
  final double height;

  FaceCoordinate({
    required this.x,
    required this.y,
    required this.width,
    required this.height,
  });

  @override
  String toString() =>
      'FaceCoordinate(x: $x, y: $y, width: $width, height: $height,)';
}
