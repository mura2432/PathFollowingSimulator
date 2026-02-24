package GVF;

public class Drivetrain {
	public enum State{
		FOLLOW_PATH,
		PID_TO_POINT,
		BRAKE,
		WAIT,
		IDLE
	} 
	public State state = State.IDLE;
	
	private Pose2d robot;
	private final double TRACK_WIDTH = 16.0;
	
	public Drivetrain (Pose2d start) {
		robot = start.clone();
	}
	
	private Path path;
	private PathData data;
	private Vector2 moveVector;
	private double turnPow;
	private PID tPID = new PID (0.5, 0, 0.005);
	
	private double correctScalar = 0.2, decelThresh = 12.0;
	
	private Pose2d targetPoint;
	private PID xPID = new PID (0.1, 0, 0.003);
	private PID yPID = new PID (0.1, 0, 0.003);
	private PID hPID = new PID (0.15, 0, 0.003);
	private double xError, yError, hError, maxPower;
	
	private double xThresh = 0.2, yThresh = 0.2, hThresh = 2.0;
	
	public void update() {
		if (path != null) {
			state = State.FOLLOW_PATH;
		}
		
		switch (state) {
		case FOLLOW_PATH:
			data = path.update(robot);
			
			if (data == null) {
				targetPoint = path.getLastPose();
				maxPower = 0.25;
				path = null;
				state = State.PID_TO_POINT;
				break;
			}
			
			Vector2 traverse = data.velocity.clone();
			Vector2 correct = new Vector2(0, traverse.mag() * traverse.mag() / data.radius * correctScalar);
			correct.rotate(Math.atan2(traverse.y, traverse.x));
			
			moveVector = Vector2.add(traverse, correct);
			moveVector.rotate(-robot.heading);
			double mag = moveVector.mag();
			moveVector.norm();
			
			double pathRot = 0;
			if (Math.abs(data.radius) < Spline.MAX_RADIUS) {
				pathRot = traverse.mag() / mag * (TRACK_WIDTH) / (2.0 * data.radius) * (data.reversed ? -1 : 1); 
			}
			
			double targetHeading = Math.atan2(traverse.y, traverse.x) + (data.reversed ? Math.PI : 0);
			turnPow = pathRot + tPID.update(targetHeading - robot.heading, -0.8, 0.8);
			
			if (data.decel & robot.getDistanceFromPoint(path.getSegLast(data.index)) < decelThresh) {
				moveVector.mul(0.2 + 0.8 * Math.sqrt(robot.getDistanceFromPoint(path.getSegLast(data.index)) / decelThresh));
			}
			moveVector.mul(data.power);
			break;
		case PID_TO_POINT:
			calculateErrors();
			PIDF();
			
			if (atPoint()) {
				state = State.BRAKE;
			}
			break;
		case BRAKE:
			moveVector = new Vector2(0, 0);
			break;
		case WAIT:
			if (!atPoint()) {
				state = State.PID_TO_POINT;
			}
			break;
		}
	}
	
	public void setPath (Path p) { path = p; }
	
	public Pose2d getPose() { return robot.clone(); }
	
    private void calculateErrors(){
        double deltaX = (targetPoint.x - robot.x);
        double deltaY = (targetPoint.y - robot.y);

        xError = Math.cos(robot.heading)*deltaX + Math.sin(robot.heading)*deltaY;
        yError = -Math.sin(robot.heading)*deltaX + Math.cos(robot.heading)*deltaY;
        hError = AngleUtil.clipAngle(targetPoint.heading - robot.heading);
    }
    
    private void PIDF(){
        double fwd = xPID.update(xError, -maxPower, maxPower);
        double strafe = yPID.update(yError, -maxPower, maxPower);
        double h = tPID.update(hError, -maxPower, maxPower);

        moveVector = new Vector2(fwd, strafe);
    }
    
    private boolean atPoint() {
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(hError) < Math.toRadians(hThresh);
    }
}
