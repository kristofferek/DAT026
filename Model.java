package bouncing_balls;

/**
 * The physics model.
 *
 * This class is where you should implement your bouncing balls model.
 *
 * The code has intentionally been kept as simple as possible, but if you wish, you can improve the design.
 *
 * @author Simon Robillard
 *
 */
class Model {

    double areaWidth, areaHeight;

    Ball [] balls;
    final double GRAVITY = -9.82;

    Model(double width, double height) {
        areaWidth = width;
        areaHeight = height;

        // Initialize the model with a few balls
        balls = new Ball[4];
        balls[0] = new Ball(width / 3, height * 0.5, 0.5, 0, 0.2);
        balls[1] = new Ball(2 * width / 3, height * 0.7, -0.5, 0, 0.3);
        balls[2] = new Ball(2 * width / 3, height * 0.2, 0.5, 0, 0.4);
    }

    void step(double deltaT) {
        // TODO this method implements one step of simulation with a step deltaT
        for (Ball b : balls) {
            // detect collision with the border
            if ((b.x < b.radius && b.vx < 0) || (b.x > areaWidth - b.radius && b.vx > 0)) {
                b.vx *= -1; // change direction of ball
            }
            if ((b.y <= b.radius && b.vy < 0) || (b.y >= areaHeight - b.radius && b.vy > 0)){
                b.vy *= -1;
            }

            for (Ball b2 : balls){
                if (hasCollided(b, b2)){
                    handleCollision(b,b2);
                }
            }

            // compute new position according to the speed of the ball
            b.x += deltaT * b.vx;
            b.y += deltaT * b.vy;
            b.vy += deltaT * GRAVITY;
        }
    }

    private boolean hasCollided(Ball b1, Ball b2){
        //Has collided when the distance between them is less than their radius combined
        double dx = b1.x - b2.x;
        double dy = b1.y - b2.y;
        double dist = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        return (dist < b1.radius + b2.radius);
    }

    private void handleCollision(Ball b1, Ball b2){
        // Calculate the basis of the collision
        double bas1[] = {b2.x - b1.x, b2.y - b1.y};
        double bas2[] = {-bas1[1], bas1[0]};
        double mat[][] = {bas1, bas2};

        //Scale the inverse basis with the velocities
        double vm1[] = multiply2DMatrix(inverse2DMatrix(mat), new double[]{b1.vx, b1.vy});
        double vm2[] = multiply2DMatrix(inverse2DMatrix(mat), new double[]{b2.vx, b2.vy});

        //Only update if they move towards each other
        if (vm1[0] > vm2[0]){
            double i = b1.mass * vm1[0] + b2.mass * vm2[0];
            double r = -(vm2[0] - vm1[0]);
            double newV1 = (i - b2.mass * r) / (b1.mass + b2.mass);
            double newV2 = r + (i - b2.mass * r) / (b1.mass + b2.mass);

            // Change basis back to the standard basis
            double ve1[] = multiply2DMatrix(mat, new double[]{newV1, vm1[1]});
            double ve2[] = multiply2DMatrix(mat, new double[]{newV2, vm2[1]});

            // Set the new velocities
            b1.vx = ve1[0];
            b1.vy = ve1[1];
            b2.vx = ve2[0];
            b2.vy = ve2[1];
        }

    }

    private static double[] multiply2DMatrix(double[][] m, double[] v) {
        return new double[]{
                m[0][0] * v[0] + m[1][0] * v[1],
                m[0][1] * v[0] + m[1][1] * v[1]
        };
    }

    private static double[][] inverse2DMatrix(double[][] m) {
        double a = m[0][0];
        double b = m[1][0];
        double c = m[0][1];
        double d = m[1][1];
        //determinant
        double det = a*d - b*c;
        double[][] inverse = {
                {
                        d/det,
                        -c/det
                },
                {
                        -b/det,
                        a/det
                }
        };
        /*    _     _
            |  d  -b |      1
            | -c   a |  *  ___   = inv  
            |_      _|     det
         */
        return inverse;
    }

    /**
     * Simple inner class describing balls.
     */
    class Ball {

        double mass;

        Ball(double x, double y, double vx, double vy, double r) {
            this.x = x;
            this.y = y;
            this.vx = vx;
            this.vy = vy;
            this.radius = r;
            this.mass = (4/3)*Math.PI*(Math.pow(r, 3))*4;
        }

        /**
         * Position, speed, and radius of the ball. You may wish to add other attributes.
         */
        double x, y, vx, vy, radius;
    }
}
