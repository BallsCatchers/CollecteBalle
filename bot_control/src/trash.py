#Append for the msg
# n_ball = [i for i in range(len(self.balls))]
# for x, y, radius, area in balls:
#     # self.get_logger().info(f"Yellow balls detected at: (x,y) = ({x},{y}) with r = {radius}")
#
#     cv2.circle(self.image, (int(x), int(y)), int(radius), (0, 0, 255), -1)
#     cv2.rectangle(self.image, (int(x) - 5, int(y) - 5), (int(x) + 5, int(y) + 5), (0, 128, 255), -1)
#
#      # Add the x and y coordinates of each ball to the ball_positions list
#     ball_positions.append(x)
#     ball_positions.append(y)
#
#     if self.balls != []:
#         far_from_all = True
#         for i in range(len(self.balls)):
#             if self.dist(x,y,self.balls[i]) < 5:
#                 print("This ball already exists !")
#                 self.balls[i] = [self.balls[i][0], x, y, radius]
#                 far_from_all = False
#                 n_ball.remove(i)
#
#         if far_from_all:
#             if len(self.balls) < len(ball_positions)//2:
#                 print("Found a new ball !")
#                 self.balls.append([time.time(), x, y, radius])
#             else:
#                 print(n_ball)
#                 if len(n_ball) == 1:
#                     print("UPDATE !")
#                     self.balls[n_ball[0]] = [self.balls[n_ball[0]][0], x, y, radius]
#
#     else:
#         self.balls.append([time.time(), x, y, radius])
