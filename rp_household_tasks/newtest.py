bridge = CvBridge()
image_subscriber = create_subscription(Image, '/camera/color/image', get_image, 10)
image_subscriber
cv_image = None