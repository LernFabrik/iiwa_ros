package application;

import java.util.Date;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.basictoolbox.container.IProvider;
import com.kuka.basictoolbox.gripper.IHandlingGripper;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
// Collaborative mode
import com.kuka.collisiondetection.abstractCollisionCondition.ICollisionCondition;
import com.kuka.collisiondetection.abstractInterruptBehavior.IInterruptBehavior;
import com.kuka.collisiondetection.abstractResumeBehavior.IResumeBehavior;
import com.kuka.collisiondetection.collisionhandler.CollisionHandler;
import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.IAnyEdgeListener;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.NotificationType;

/**
 * Moves the LBR in a start position, creates an FRI-Session and executes a
 * PositionHold motion with FRI overlay. During this motion joint angles and
 * joint torques can be additionally commanded via FRI.
 */
public class FRIOverlayGripper extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private Tool _toolAttached;
    private IHandlingGripper _gripper;
    private Workpiece _workPiece = null;
    /*private LoadData _loadData;
    
    
    // Tool Data
    private static final String TOOL_NAME = "MyVirtualGripper";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100};
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = {0, 0, 100};*/
    
    // Collaborative mode
    private ICollisionCondition collisionCondition;
    private ICondition iCondition;
    private ConditionObserver conObserver;
    private IInterruptBehavior iInterruptBehavior;
    private IResumeBehavior iResumeBehavior;
    public boolean collision = true;
    private CollisionHandler collisionHandler;
    public double threshold = 30;
    
    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "192.170.10.1";
        //_clientName = "172.31.1.140";
        //_workPiece.setName("universal_product");
        // attach a gripper
        
        // Collaborative mode
        iCondition = ForceCondition.createSpatialForceCondition(_lbr.getFlange(), threshold);
        conObserver = getObserverManager().createAndEnableConditionObserver(iCondition, NotificationType.OnEnable, new IAnyEdgeListener() {
			
			@Override
			public void onAnyEdge(ConditionObserver conditionObserver, Date time,
					int missedEvents, boolean conditionValue) {
				if(conditionValue){
					if(collision){
						getLogger().info("Collison State");
						getApplicationControl().setApplicationOverride(0.0d);
						ThreadUtil.milliSleep(500);
					}else{
						getApplicationControl().setApplicationOverride(0.25d);
						getLogger().info("Resuming the motion");
					}
				}else{
					getLogger().info("No Collision is detected -> !I AM IN HAPPY STATE!");
					getApplicationControl().setApplicationOverride(0.25d);
				}
				
			}
		});
        
        _workPiece = getApplicationData().createFromTemplate("universal_product");
        _workPiece.attachTo(_lbr.getFlange());
        _lbr.setSafetyWorkpiece(_workPiece);
        _toolAttached = getApplicationData().createFromTemplate("SchunkW");
        _toolAttached.attachTo(_lbr.getFlange());
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        // for torque mode, there has to be a command value at least all 5ms
        friConfiguration.setSendPeriodMilliSec(5);
        friConfiguration.setReceiveMultiplier(1);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");
        
        ClientCommandMode mode = ClientCommandMode.POSITION;
        
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession, mode);
        
        double stiffness = 500.;
        
        // start PositionHold with overlay
        JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(stiffness, stiffness, stiffness, stiffness, stiffness, stiffness, stiffness);
        PositionControlMode posMode = new PositionControlMode();
        if (mode == ClientCommandMode.TORQUE)
        	ctrMode.setDampingForAllJoints(0.);
        
        PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);
        
        // PositionHold posHold = new PositionHold(posMode, -1, TimeUnit.SECONDS);
        
        getLogger().info("ROS enabled motion is started");

        _toolAttached.move(posHold.addMotionOverlay(jointOverlay));

        // done
        friSession.close();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final FRIOverlayGripper app = new FRIOverlayGripper();
        app.runApplication();
    }

}
