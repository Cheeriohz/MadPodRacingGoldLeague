using System;
using System.Linq;
using System.IO;
using System.Text;
using System.Collections;
using System.Collections.Generic;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
public class Player
{
    
    public static void Main(string[] args)
    {
        string[] inputs;
        int laps = int.Parse(Console.ReadLine());
        int checkpointCount = int.Parse(Console.ReadLine());
        (int x, int y)[] checkpoints = new (int x, int y)[checkpointCount];
        for (int i = 0; i < checkpointCount; i++)
        {
            inputs = Console.ReadLine().Split(' ');
            int checkpointX = int.Parse(inputs[0]);
            int checkpointY = int.Parse(inputs[1]);
            checkpoints[i] = (checkpointX, checkpointY);
        }

        Dictionary<int, NodeLookup> nodes = Utility.NodeBuild(checkpoints);
        OverSeer overSeer = new OverSeer(nodes);
        Racing racing = new Racing(nodes);
        Interference interference = new Interference(nodes);

        // game loop
        while (true)
        {
            InputData[] racerInputs = new InputData[4];
            for (int i = 0; i < 2; i++)
            {
                inputs = Console.ReadLine().Split(' ');
                int x = int.Parse(inputs[0]); // x position of your pod
                int y = int.Parse(inputs[1]); // y position of your pod
                int vx = int.Parse(inputs[2]); // x speed of your pod
                int vy = int.Parse(inputs[3]); // y speed of your pod
                int angle = int.Parse(inputs[4]); // angle of your pod
                int nextCheckPointId = int.Parse(inputs[5]); // next check point id of your pod

                racerInputs[i] = new InputData
                {
                    X = x,
                    Y = y,
                    Vx = vx,
                    Vy = vy,
                    Angle = angle,
                    NextCheckPointId = nextCheckPointId
                };
            }
            for (int i = 0; i < 2; i++)
            {
                inputs = Console.ReadLine().Split(' ');
                int x2 = int.Parse(inputs[0]); // x position of the opponent's pod
                int y2 = int.Parse(inputs[1]); // y position of the opponent's pod
                int vx2 = int.Parse(inputs[2]); // x speed of the opponent's pod
                int vy2 = int.Parse(inputs[3]); // y speed of the opponent's pod
                int angle2 = int.Parse(inputs[4]); // angle of the opponent's pod
                int nextCheckPointId2 = int.Parse(inputs[5]); // next check point id of the opponent's pod

                racerInputs[i+2] = new InputData
                {
                    X = x2,
                    Y = y2,
                    Vx = vx2,
                    Vy = vy2,
                    Angle = angle2,
                    NextCheckPointId = nextCheckPointId2
                };
            }

            // Handle Overseer
            overSeer.OverSee(racerInputs);

            // Handle Racer
            (int goToX, int goToY, string thrust) = racing.Race(racerInputs[0].X, racerInputs[0].Y, racerInputs[0].Vx, racerInputs[0].Vy, racerInputs[0].NextCheckPointId, racerInputs[0].Angle.NormalizeAngleTo360(), racerInputs[2], racerInputs[3]);
            Console.WriteLine($"{goToX} {goToY} {thrust}");
            
            // Handle Interference
            (goToX, goToY, thrust) = interference.Interfere(racerInputs[1].X, racerInputs[1].Y, racerInputs[1].Vx, racerInputs[1].Vy, racerInputs[1].Angle.NormalizeAngleTo360(), overSeer.bullyTarget, racerInputs, overSeer.lapsCompleted);
            
            Console.WriteLine($"{goToX} {goToY} {thrust}");
        }
    }    
}


class OverSeer
{
    public OverSeer(Dictionary<int, NodeLookup> nodes) {
        this.CheckPoints = nodes;
        this.racerData = new RacerData[4];

        for (int i = 0; i < this.racerData.Length; i++)
        { 
            this.racerData[i] = new RacerData
            {
                checkPointCount = 0,
                currentCheckpointId = 0
            };
        }

    }

    private Dictionary<int, NodeLookup> CheckPoints;
    private RacerData[] racerData;
    public int bullyTarget = 2;
    public int lapsCompleted;
    public void OverSee(InputData[] inputs)
    {
        this.MonitorRaceStatus(inputs);
        this.UpdateHighLevelStrategy();
    }

    private void UpdateHighLevelStrategy()
    {
        int currentBullyTarget = this.bullyTarget;
        int packLeader 
            = this.racerData[2].checkPointCount == this.racerData[3].checkPointCount 
                ? this.bullyTarget
                : this.racerData[2].checkPointCount > this.racerData[3].checkPointCount 
                    ? 2 
                    : 3;

        int checkPointDelta = Math.Abs(this.racerData[3].checkPointCount - this.racerData[2].checkPointCount);
        
        if(checkPointDelta < 1)
        {
            this.bullyTarget = currentBullyTarget;
        }
        else
        {
            this.bullyTarget = packLeader;
        }

    }

    private void MonitorRaceStatus(InputData[] inputs)
    {
        for (int i = 0; i < inputs.Length; i++)
        {
            this.racerData[i] = this.MonitorRacer(this.racerData[i], inputs[i]);
            if(this.racerData[i].checkPointCount > this.CheckPoints.Count * (this.lapsCompleted + 1))
            {
                this.lapsCompleted++;
            }
        }

    }

    private RacerData MonitorRacer(RacerData dataForRacer, InputData racerInput)
    {
        if(dataForRacer.currentCheckpointId != racerInput.NextCheckPointId)
        {
            return new RacerData
            {
                currentCheckpointId = racerInput.NextCheckPointId,
                checkPointCount = dataForRacer.checkPointCount + 1
            };
        }
        return dataForRacer;
    }
}

class Racing {

    public Racing(Dictionary<int, NodeLookup> nodes) {
         this.CheckPoints = nodes;
    }

    #region Consts and Enums
    protected const string BOOST = "BOOST";
    protected const string SHIELD = "SHIELD";
    protected const int DEFAULT_CORRECTION_THRESHOLD_SPEED = 250;

    public const int BURN_HARD = 200;
    public const int BURN_STEADY = 150;
    public const int BURN_SLOW = 35;
    public const int BURN_STALL = 15;

    //public const int BURN_STEADY = 100;
    //public const int BURN_SLOW = 75;
    //public const int BURN_STALL = 15;


    private const int BOOST_PLAY_OUT_TURN_COUNT = 5;

    public const int APPROACH_DISTANCE = 500;

    enum RacingStrategy {
        BOOSTING,
        BOOSTING_PLAY_OUT,
        
        BURN_TOWARD,
        BURN_NEXT,
        BURN_APPROACH,
        STALL_PLAY_OUT
    };

    enum LegStatus {
        FAR,
        MID,
        NEAR
    };
    #endregion

    #region Props and Fields
    int currentCheckPointId = -1, legDistanceX = -1, legDistanceY = -1, boostPlayRounds = 0;

    int nextCheckX { get => this.CheckPoints[this.currentCheckPointId].NextX; }
    int nextCheckY { get => this.CheckPoints[this.currentCheckPointId].NextY; }
    int currentCheckX { get => this.CheckPoints[this.currentCheckPointId].CheckpointX; }
    int currentCheckY { get => this.CheckPoints[this.currentCheckPointId].CheckpointY; }
    int currentApproachX { get => this.CheckPoints[this.currentCheckPointId].CheckpointX; }
    int currentApproachY { get => this.CheckPoints[this.currentCheckPointId].CheckpointY; }
    int currentLegDist { get => this.CheckPoints[this.currentCheckPointId].LegDistance; }

    RacingStrategy raceStrategy = RacingStrategy.BOOSTING;
    LegStatus legStatus = LegStatus.FAR;
    private bool initialized = false, haveBoosted = false;

    protected Dictionary<int, NodeLookup> CheckPoints;
    #endregion

    public (int, int, string) Race(int x, int y, int vx, int vy, int checkPointId, int currentAngle, InputData enemyA, InputData enemyB) 
    {
        if(!initialized || CheckPointChanged(checkPointId)) {
           this.HandleCheckPointChange(x, y, checkPointId);
        }

        int nextCheckpointAngle180 = Utility.DetermineRotationalAngleToPoint(x, y, this.CheckPoints[this.currentCheckPointId].CheckpointX, this.CheckPoints[this.currentCheckPointId].CheckpointY, currentAngle);

        int nextDestinationDistance = this.ComputeLegStatus(x, y);
        this.ComputeStrategy(x, y, vx, vy, nextCheckpointAngle180);

        this.RecordStatus(x, y, vx, vy, nextCheckpointAngle180);
        (int xTarget, int yTarget) = this.DetermineTargetPoint(x, y, vx, vy, nextDestinationDistance, nextCheckpointAngle180);

        this.initialized = true;
        return (xTarget, yTarget, DetermineThrust(x, y, vx, vy, currentAngle, nextCheckpointAngle180, nextDestinationDistance, enemyA, enemyB, xTarget, yTarget));
    }

    private void RecordStatus(int x, int y, int vx, int vy, int nextCheckpointAngle) 
    {        
        Console.Error.WriteLine($"Strategy: {this.raceStrategy.ToString("G")} | Leg Status: {this.legStatus.ToString("G")} | VelocityX {vx} | VelocityY {vy} | CheckPoint {this.currentCheckPointId}");    
    }

    #region Leg Status and Strategy
    private int ComputeLegStatus(int x, int y) {
        int nextDestinationDistance = Utility.CalculateDistanceAsInt(
            x, 
            y, 
            this.raceStrategy switch 
                {
                    RacingStrategy.BURN_APPROACH => this.currentApproachX,
                    _ => this.currentCheckX
                },
            this.raceStrategy switch
                {
                    RacingStrategy.BURN_APPROACH => this.currentApproachY,
                    _ => this.currentCheckY
                });
        
        if(this.legStatus == LegStatus.FAR) {
            if(this.NearCheckPoint(nextDestinationDistance / 2)) {
                Console.Error.WriteLine("Now Near CheckPoint");
                this.legStatus = LegStatus.MID;
                return nextDestinationDistance;
            }
        }

        if(this.legStatus == LegStatus.MID) {
            if(this.NearCheckPoint(nextDestinationDistance)) {
                Console.Error.WriteLine("Now Near CheckPoint");
                this.legStatus = LegStatus.NEAR;
                return nextDestinationDistance;
            }
        }
        

        if(this.legStatus == LegStatus.NEAR) {
            if(!this.NearCheckPoint(nextDestinationDistance)){
                Console.Error.WriteLine("No Longer Near CheckPoint");
                this.legStatus = LegStatus.FAR;
                return nextDestinationDistance;
            }
        }

        return nextDestinationDistance;
    }

    private void ComputeStrategy(int x, int y, int vx, int vy, int nextCheckPointAngle) {
        // After we use the boosting strategy we want to let the car play out the boost so as not to try to course correct for speed we will quickly burn off
        if(this.raceStrategy == RacingStrategy.BOOSTING) {
            if(Utility.CurrentVelocityWillHitAndOvershootTarget(x, y, vx, vy, this.currentCheckX, this.currentCheckY, DetermineOvershootTargetForCurrentCheckpoint(x, y, vx, vy)))
            {
                this.raceStrategy = RacingStrategy.STALL_PLAY_OUT;
            }
            else
            {
                this.raceStrategy = RacingStrategy.BOOSTING_PLAY_OUT;
            }
        }
        else if(this.raceStrategy == RacingStrategy.BOOSTING_PLAY_OUT) {
            
            if(Utility.CurrentVelocityWillHitAndOvershootTarget(x, y, vx, vy, this.currentCheckX, this.currentCheckY, DetermineOvershootTargetForCurrentCheckpoint(x, y, vx, vy)))
            {
                this.raceStrategy = RacingStrategy.STALL_PLAY_OUT;
            }
            else if(this.boostPlayRounds++ > BOOST_PLAY_OUT_TURN_COUNT) {
                this.raceStrategy = RacingStrategy.BURN_TOWARD;         
                this.boostPlayRounds = 0;
            }
        }
        
        // Intentionally allowing the if to fall into the subsequent else if in case BURN_TOWARD is no longer the right pick
        if(this.raceStrategy == RacingStrategy.BURN_TOWARD)
        { 
            //if(this.legStatus == LegStatus.NEAR) 
            //{
            //    int positionalAngleToNextCheckPoint = Utility.DeterminePositionAngleToPoint(x, y, this.nextCheckX, this.nextCheckY);
            //    int determineVelocityAngle = Math.Atan2(vy, vx).ToDegrees();

            //    if(Math.Abs(positionalAngleToNextCheckPoint - determineVelocityAngle) < 15)
            //    {
            //        this.raceStrategy = RacingStrategy.BURN_NEXT;
            //    }
            //}
            
            if(Utility.CurrentVelocityWillHitAndOvershootTarget(x, y, vx, vy, this.currentCheckX, this.currentCheckY, DetermineOvershootTargetForCurrentCheckpoint(x, y, vx, vy)))
            { 
                this.raceStrategy = RacingStrategy.STALL_PLAY_OUT;
            }
        }
        else if(this.raceStrategy == RacingStrategy.STALL_PLAY_OUT)
        {
            if(!Utility.CurrentVelocityWillHitAndOvershootTarget(x, y, vx, vy, this.currentCheckX, this.currentCheckY, 205))
            {
                this.raceStrategy = RacingStrategy.BURN_TOWARD;
            }
        }
        else if(this.legStatus == LegStatus.FAR && this.raceStrategy == RacingStrategy.BURN_NEXT) {
            this.raceStrategy = RacingStrategy.BURN_APPROACH;
        }
        else if(this.legStatus == LegStatus.MID && this.raceStrategy == RacingStrategy.BURN_APPROACH) {
            this.raceStrategy = RacingStrategy.BURN_TOWARD;
        }

        
        
    }

    private double DetermineOvershootTargetForCurrentCheckpoint(int x, int y, int vx, int vy)
    {
        int distanceFromCheckpointToNextCheckPoint = Utility.CalculateDistanceAsInt(this.currentCheckX, this.currentCheckY, this.nextCheckX, this.nextCheckY);
        if(distanceFromCheckpointToNextCheckPoint < 2550)
        {
            //Console.Error.WriteLine($"overShootDetermination exited early due to close check points");
            return 850;
        }
        
        int positionalAngleToNextCheckPoint = Utility.DeterminePositionAngleToPoint(x, y, this.nextCheckX, this.nextCheckY).NormalizeAngleTo360();
        int velocityAngle = Math.Atan2(vy, vx).ToDegrees().NormalizeAngleTo360();

        int currentDeltaAngle = (positionalAngleToNextCheckPoint - velocityAngle).NormalizeAngleTo180();

        double overshootValue;

        if(currentDeltaAngle > 170)
        {
            overshootValue = 750;
        }
        else if (currentDeltaAngle > 155)
        {
            overshootValue = 850;
        }
        else if (currentDeltaAngle > 125)
        {
            overshootValue = 950;
        }
        else if (currentDeltaAngle > 75)
        {
            overshootValue = 1200;
        }
        else if (currentDeltaAngle > 55)
        {
            overshootValue = 1300;
        }
        else
        {
            overshootValue = 3000;
        }


        //Console.Error.WriteLine($"pos {positionalAngleToNextCheckPoint} | vel {velocityAngle} | delta {currentDeltaAngle} | overshoot {overshootValue}");
        return overshootValue;
    }

    private bool NearCheckPoint(int distanceRemaining) 
    {
        return this.raceStrategy switch 
        {
            RacingStrategy.BURN_APPROACH => NearCheckPointApproach(distanceRemaining),
            _ => NearCheckPointDirect(distanceRemaining)
        };
    }

    private bool NearCheckPointApproach(int distanceRemaining) {
        //Console.Error.WriteLine($"appoachNearDistanceRemaing {distanceRemaining} | legDistance {this.currentLegDist} | toBeat {(this.currentLegDist * 5 / 10)}");
        return distanceRemaining < this.CheckPoints[this.currentCheckPointId].MidDistance;
    }

    private bool NearCheckPointDirect(int distanceRemaining) 
    {

        return distanceRemaining < this.CheckPoints[this.currentCheckPointId].NearDistance;
    }
    #endregion

    private (int x, int y) DetermineTargetPoint(int x, int y, int vx, int vy,  int distanceToNextCheckPoint, int nextCheckpointAngle) {
        Utility.LocationDelegate fallbackDelegate = () => (this.DetermineX(), this.DetermineY());
        if(this.raceStrategy == RacingStrategy.BURN_TOWARD)
        {
            //Console.Error.WriteLine($"Racer nextCheckAngle {nextCheckpointAngle} | p ({x},{y}) | v ({vx},{vy})");
            return this.DetermineCorrectedTargetPoint(x, y, this.currentCheckX, this.currentCheckY,  vx, vy, distanceToNextCheckPoint, nextCheckpointAngle, fallbackDelegate, DEFAULT_CORRECTION_THRESHOLD_SPEED, 1.05);
        }
        else if (this.raceStrategy == RacingStrategy.BURN_APPROACH)
        {
            return this.DetermineCorrectedTargetPoint(x, y, this.currentApproachX, this.currentApproachY, vx, vy, distanceToNextCheckPoint, 0, fallbackDelegate, DEFAULT_CORRECTION_THRESHOLD_SPEED, 1.05);
        }
        return fallbackDelegate.Invoke();
    }

    protected (int x, int y) DetermineCorrectedTargetPoint(int x, int y, int destinationX, int destinationY, int vx, int vy, int distanceToNextCheckPoint, int nextCheckpointAngle, Utility.LocationDelegate fallback, int correctionSpeedThreshold, double compensationFactor) {
        // This correction math is done for overshooting due to a high velocity. Only apply when going fast enough to matter and car is accelerating in some part towards the goal already
        if(nextCheckpointAngle < -45 && nextCheckpointAngle > 45  || ( (Math.Abs(vx) + Math.Abs(vy)) < correctionSpeedThreshold )) {
            return fallback.Invoke();//(this.DetermineX(), this.DetermineY());
        }

        // Need to compute velocity projected location compared which is not the angle given by the car facing.
        double velocityAngle = Math.Atan2(vy, vx);

        double arrivalX = x + (distanceToNextCheckPoint * Math.Cos(velocityAngle));
        double arrivalY = y + (distanceToNextCheckPoint * Math.Sin(velocityAngle));
        double offsetX = Math.Abs(destinationX - arrivalX) * compensationFactor;
        double offsetY = Math.Abs(destinationY - arrivalY) * compensationFactor;

        //Console.Error.WriteLine($"VelocityAngle {velocityAngle} | arrivalX {arrivalX} | arrivalY {arrivalY} | offsetX {offsetX} | offsetY {offsetY}");

        if(arrivalX < destinationX && arrivalY < destinationY) {
            //Console.Error.WriteLine("Correcting for top leftward trajectory");
            return (destinationX + (int)(Math.Floor(offsetX)), destinationY + (int)(Math.Floor(offsetY)));
        }

        if(arrivalX >= destinationX && arrivalY < destinationY) {
            //Console.Error.WriteLine("Correcting for top rightward trajectory");
            return (destinationX - (int)(Math.Floor(offsetX)), destinationY + (int)(Math.Floor(offsetY)));
        }

        if(arrivalX < destinationX && arrivalY > destinationY) {
            //Console.Error.WriteLine("Correcting for bottom leftward trajectory");
            return (destinationX + (int)(Math.Floor(offsetX)), destinationY - (int)(Math.Floor(offsetY)));
        }

        //Console.Error.WriteLine("Correcting for bottom rightward trajectory");
        return (destinationX - (int)(Math.Floor(offsetX)), destinationY - (int)(Math.Floor(offsetY)));
    }

    private int DetermineX() {
        return this.raceStrategy switch {
            RacingStrategy.BOOSTING => throw new Exception("Boost determination is made after the position determination"),
            RacingStrategy.BOOSTING_PLAY_OUT => this.currentCheckX,
            RacingStrategy.BURN_TOWARD => this.currentCheckX,
            RacingStrategy.STALL_PLAY_OUT => this.CheckPoints[this.currentCheckPointId].NextX,
            RacingStrategy.BURN_NEXT => this.CheckPoints[this.currentCheckPointId].NextX,
            RacingStrategy.BURN_APPROACH => this.CheckPoints[this.currentCheckPointId].ApproachX,
            _ => 0
        };
        
    }

    private int DetermineY() {
        return this.raceStrategy switch {
            RacingStrategy.BOOSTING => throw new Exception("Boost determination is made after the position determination"),
            RacingStrategy.BOOSTING_PLAY_OUT => this.currentCheckY,
            RacingStrategy.BURN_TOWARD => this.currentCheckY,
            RacingStrategy.STALL_PLAY_OUT => this.CheckPoints[this.currentCheckPointId].NextY,
            RacingStrategy.BURN_NEXT => this.CheckPoints[this.currentCheckPointId].NextY,
            RacingStrategy.BURN_APPROACH => this.CheckPoints[this.currentCheckPointId].ApproachY,
            _ => 0
        };
        
    }

    private string DetermineThrust(int x, int y, int vx, int vy, int currentAbsoluteAngle, int currentAngleToGoal, int distanceToCheckPoint, InputData enemyA, InputData enemyB, int xTarget, int yTarget)
    {
        bool enemyACollisionImminent = Utility.IsCollisionImminent(x, y, vx, vy, enemyA.X, enemyA.Y, enemyA.Vx, enemyA.Vy);
        bool enemyACollisionRequiresShield = enemyACollisionImminent && Utility.IsCollisionBad(x, y, vx, vy, enemyA.X, enemyA.Y, enemyA.Vx, enemyA.Vy, xTarget, yTarget);
        bool enemyBCollisionImminent = enemyACollisionRequiresShield || Utility.IsCollisionImminent(x, y, vx, vy, enemyB.X, enemyB.Y, enemyB.Vx, enemyB.Vy);
        bool enemyBCollisionRequiresShield = enemyBCollisionImminent && Utility.IsCollisionBad(x, y, vx, vy, enemyA.X, enemyA.Y, enemyA.Vx, enemyA.Vy, xTarget, yTarget);
        if (vx + vy > 300 && (enemyACollisionRequiresShield || enemyBCollisionRequiresShield))
        { 
            return SHIELD;
        }


        return this.raceStrategy switch {
            RacingStrategy.BOOSTING => throw new Exception("Should not be able to determine thrust during boosting stage"),
            RacingStrategy.BOOSTING_PLAY_OUT => "200",
            RacingStrategy.STALL_PLAY_OUT => "0",
            RacingStrategy.BURN_TOWARD 
                => this.CanBoost() && this.ShouldBoost(x, y, currentAngleToGoal, distanceToCheckPoint) 
                    ? BOOST 
                    : this.DetermineThrustForBurnTowardsAngleAware(currentAngleToGoal, distanceToCheckPoint).ToString(),
            RacingStrategy.BURN_NEXT => this.DetermineThrustForBurnNext(currentAbsoluteAngle).ToString(),
            _ => string.Empty
        };
        
    }

    private int DetermineThrustForBurnNext(int currentAbsoluteAngle) {
        int subsequentCheckPointAngleDelta = this.CheckPoints[this.currentCheckPointId].AbsoluteAngleToCheckPoint - currentAbsoluteAngle;

        if(subsequentCheckPointAngleDelta > 15 || subsequentCheckPointAngleDelta < -15) {
            return BURN_HARD;
        }

        return this.CheckPoints[this.currentCheckPointId].BurnFactor;
    }


    private int DetermineThrustForBurnTowardsAngleAware(int currentAngleToGoal, int distanceToCheckPoint) {

        if(distanceToCheckPoint < 1500)
        {
            if(currentAngleToGoal < -35 || currentAngleToGoal > 35)
            {
                return 0;
            }
        }

        if(currentAngleToGoal > 160 || currentAngleToGoal < -160) {
            return BURN_SLOW;
        }

        if(currentAngleToGoal > 90 || currentAngleToGoal < -90) {
            return BURN_STEADY;
        }

        return BURN_HARD;
    }

    private bool CanBoost()
    {
        return !this.haveBoosted;
    }

    private bool ShouldBoost(int x, int y, int currentAngleToGoal, int distanceToCheckPoint) {
        bool alignmentAndDistanceAreGood = currentAngleToGoal < 10 && currentAngleToGoal > -10 && distanceToCheckPoint > 3500;
        if(alignmentAndDistanceAreGood){

            
            int angleToCheckPoint360 = Utility.DeterminePositionAngleToPoint(x, y, this.currentCheckX, this.currentCheckY).NormalizeAngleTo360();
            int angleToSubsequentCheckPointFromCurrentCheckPoint360 = Utility.DeterminePositionAngleToPoint(this.currentCheckX, this.currentCheckY, this.nextCheckX, this.nextCheckY).NormalizeAngleTo360();

            int minimumAngleBetweenCheckPointAtCurrentTrajectoryAndSubsequentCheckPoint = Utility.CalculateMinimum180AngleBetweenTwo360Angles(angleToCheckPoint360, angleToSubsequentCheckPointFromCurrentCheckPoint360);

            if(minimumAngleBetweenCheckPointAtCurrentTrajectoryAndSubsequentCheckPoint < 40)
            {
                this.haveBoosted = true;
                this.raceStrategy = RacingStrategy.BOOSTING;
                return true;
            }
        }
        return false;
    }

    private bool CheckPointChanged(int checkpointId ) => this.currentCheckPointId != checkpointId;

    private void HandleCheckPointChange(int x, int y, int newCheckpointId) {
        //Console.Error.WriteLine("CheckPoint Change Detected");
        this.currentCheckPointId = newCheckpointId;

        this.legDistanceX = Math.Abs(this.CheckPoints[this.currentCheckPointId].NextX - this.CheckPoints[this.currentCheckPointId].CheckpointX);
        this.legDistanceY = Math.Abs(this.CheckPoints[this.currentCheckPointId].NextY - this.CheckPoints[this.currentCheckPointId].CheckpointY);

        this.raceStrategy = RacingStrategy.BURN_APPROACH;
        this.legStatus = LegStatus.FAR;
    }

}

class Interference : Racing
{
    public Interference(Dictionary<int, NodeLookup> nodes) : base(nodes) { }

    enum InterferenceStrategy {
        RELOCATE_TO_INTERCEPT,
        HOLD,
        TACKLE,
        ZONE_BLOCK
    };

    private int currentTargetId;
    private int relocationCheckPointId = -1;
    private int distanceToTarget = -1;
    private InterferenceStrategy currentInterferenceStrategy = InterferenceStrategy.RELOCATE_TO_INTERCEPT;

    public (int, int, string) Interfere(int x, int y, int vx, int vy, int currentAngle, int targetId, InputData[] inputs, int lapsCompleted)
    {
        if (this.currentTargetId != targetId)
        {
            HandleTargetChange(x, y, targetId, inputs[targetId]);
        }

        this.currentInterferenceStrategy = DetermineInterferenceStrategy(x, y, vx, vy, inputs[targetId]);
        (int oX, int oY, string thrust) = DetermineOutputs(x, y, vx, vy, currentAngle, inputs, lapsCompleted);

        ClearDataForEndCompute();

        return (oX, oY, thrust);
    }

    private void HandleTargetChange(int x, int y, int newTarget, InputData targetData)
    {
        this.currentTargetId = newTarget;

        this.relocationCheckPointId = Utility.DetermineRelocationCheckpointForIntercept(this.CheckPoints, x, y, targetData);
        this.currentInterferenceStrategy = InterferenceStrategy.RELOCATE_TO_INTERCEPT;
        
    }

    #region Strategy Management

    private InterferenceStrategy DetermineInterferenceStrategy(int x, int y, int vx, int vy, InputData targetData) 
    {
        return this.currentInterferenceStrategy switch 
        {
            InterferenceStrategy.RELOCATE_TO_INTERCEPT => DetermineInterferenceStrategyWhileRelocatingToIntercept(x, y, vx, vy, targetData),
            InterferenceStrategy.HOLD => DetermineInterferenceStrategyWhileHolding(x, y, vx, vy, targetData),
            InterferenceStrategy.TACKLE => DetermineInterferenceStrategyWhileTackling(x, y, vx, vy, targetData),
            InterferenceStrategy.ZONE_BLOCK => DetermineInterferenceStrategyWhileZoneBlocking(x, y, vx, vy, targetData),
            _ =>  InterferenceStrategy.RELOCATE_TO_INTERCEPT
        };
    }

    private InterferenceStrategy DetermineInterferenceStrategyWhileRelocatingToIntercept(int x, int y, int vx, int vy, InputData targetData) 
    {

        int interceptCost = Utility.BallparkTraversalValue(x, y, this.CheckPoints[this.relocationCheckPointId].CheckpointX, this.CheckPoints[this.relocationCheckPointId].CheckpointY);
        int targetCost = Utility.BallparkTraversalValue(targetData.X, targetData.Y, this.CheckPoints[this.relocationCheckPointId].CheckpointX, this.CheckPoints[this.relocationCheckPointId].CheckpointY);

        if(targetCost < interceptCost)
        {
            if(targetData.NextCheckPointId == this.relocationCheckPointId)
            {
                return InterferenceStrategy.TACKLE;
            }
        }

        if(Utility.CurrentVelocityWillHitAndOvershootTarget(x, y, vx, vy, this.CheckPoints[relocationCheckPointId].InteferenceX, this.CheckPoints[relocationCheckPointId].InteferenceY, 850, 1000))
        {
            return InterferenceStrategy.HOLD;
        }
        return InterferenceStrategy.RELOCATE_TO_INTERCEPT;
    }

    
    private InterferenceStrategy DetermineInterferenceStrategyWhileHolding(int x, int y, int vx, int vy, InputData targetData)
    {
        this.ComputeDistanceToTarget(x, y, targetData);
        if(this.distanceToTarget < 10000 && targetData.NextCheckPointId == this.relocationCheckPointId)
        {
            return InterferenceStrategy.ZONE_BLOCK;
        }

        int distanceToRelocationCheckPoint = Utility.CalculateDistanceAsInt(x, y, this.CheckPoints[this.relocationCheckPointId].InteferenceX, this.CheckPoints[relocationCheckPointId].InteferenceY);
        if(distanceToRelocationCheckPoint > 1400 && !Utility.CurrentVelocityWillHitAndOvershootTarget(x, y, vx, vy, this.CheckPoints[relocationCheckPointId].InteferenceX, this.CheckPoints[relocationCheckPointId].InteferenceY, 850, 1000))
        {
            // Not updating the checkpoint, just heading back to the checkpoint we are holding on
            return InterferenceStrategy.RELOCATE_TO_INTERCEPT;
        }

        return InterferenceStrategy.HOLD;
    }

    private InterferenceStrategy DetermineInterferenceStrategyWhileTackling(int x, int y, int vx, int vy, InputData targetData) 
    {
        this.ComputeDistanceToTarget(x, y, targetData);
        if (this.distanceToTarget >= 3500)
        {
            this.relocationCheckPointId = Utility.DetermineRelocationCheckpointForIntercept(this.CheckPoints, x, y, targetData);
            return InterferenceStrategy.RELOCATE_TO_INTERCEPT;
        }

        if (this.distanceToTarget < 2000) 
        {
            if(this.CanZoneBlock(x, y, vx, vy, targetData, targetData.NextCheckPointId))
            {
                return InterferenceStrategy.ZONE_BLOCK;
            }
        }
        return InterferenceStrategy.TACKLE;
    }

    private InterferenceStrategy DetermineInterferenceStrategyWhileZoneBlocking(int x, int y, int vx, int vy, InputData targetData) 
    {
        if((targetData.NextCheckPointId != this.relocationCheckPointId && this.relocationCheckPointId != -1)
            || !CanZoneBlock(x, y, vx, vy, targetData, targetData.NextCheckPointId, -300))
        {
            if(this.CanZoneBlock(x, y, vx, vy, targetData, targetData.NextCheckPointId))
            {
                this.relocationCheckPointId = targetData.NextCheckPointId;
                return InterferenceStrategy.ZONE_BLOCK;
            }

            this.relocationCheckPointId = Utility.DetermineRelocationCheckpointForIntercept(this.CheckPoints, x, y, targetData);
            return InterferenceStrategy.RELOCATE_TO_INTERCEPT; 
        }

        return InterferenceStrategy.ZONE_BLOCK;
    }

    private bool CanZoneBlock(int x, int y, int vx, int vy, InputData targetData, int checkPointToBlock, int interceptCostMod = 0)
    {
        //if(vx + vy < 400 && targetData.Vx + targetData.Vy < 400)
        //{
            int interceptCost = Utility.BallparkTraversalValue(x, y, this.CheckPoints[checkPointToBlock].CheckpointX, this.CheckPoints[checkPointToBlock].CheckpointY);
            int targetCost = Utility.BallparkTraversalValue(targetData.X, targetData.Y, this.CheckPoints[checkPointToBlock].CheckpointX, this.CheckPoints[checkPointToBlock].CheckpointY);

            if(interceptCost + interceptCostMod < targetCost)
            {
                return true;
            }
        //}
        return false;
    }

    #endregion

    #region Outputs Management
    private (int, int, string) DetermineOutputs(int x, int y, int vx, int vy, int currentAngle, InputData[] inputs, int lapsCompleted) 
    {
        if(this.currentInterferenceStrategy == InterferenceStrategy.RELOCATE_TO_INTERCEPT)
        {
            return this.DetermineOutputsForRelocation(x, y, vx, vy, currentAngle, inputs, lapsCompleted);
        }

        if(this.currentInterferenceStrategy == InterferenceStrategy.HOLD)
        {
            return ( inputs[this.currentTargetId].X, inputs[this.currentTargetId].Y, (0).ToString() );
        }

        if(this.currentInterferenceStrategy == InterferenceStrategy.TACKLE)
        {
            return this.DetermineOutputsForTackle(x, y, vx, vy, currentAngle, inputs[this.currentTargetId], lapsCompleted);
        }

        if(this.currentInterferenceStrategy == InterferenceStrategy.ZONE_BLOCK)
        {
            return this.DetermineOutputsForZoneBlock(x, y, vx, vy, currentAngle, inputs, lapsCompleted);
        }

        return (inputs[this.currentTargetId].X, inputs[this.currentTargetId].Y, (BURN_HARD).ToString());
    }

    private (int x, int y, string thrust) DetermineOutputsForRelocation(int x, int y, int vx, int vy, int currentAngle, InputData[] inputs, int lapsCompleted)
    {
        int destinationX = this.CheckPoints[this.relocationCheckPointId].InteferenceX;
        int destinationY = this.CheckPoints[this.relocationCheckPointId].InteferenceY;
        int nextCheckpointAngle = Utility.DetermineRotationalAngleToPoint(x, y, destinationX, destinationY, currentAngle);
        int distanceToCheckPoint = Utility.CalculateDistanceAsInt(x, y, destinationX, destinationY);

        Utility.LocationDelegate fallbackDelegate = () => (destinationX, destinationY);
        (int oX, int oY) = this.DetermineCorrectedTargetPoint(x, y, destinationX, destinationY, vx, vy, distanceToCheckPoint, nextCheckpointAngle, fallbackDelegate, DEFAULT_CORRECTION_THRESHOLD_SPEED, 1.05);
        
        return CheckForPartnerCollision(oX, oY, x, y, vx, vy, inputs[0]);
    }

    private (int oX, int oY, string thrust) CheckForPartnerCollision(int oX, int oY, int x, int y, int vx, int vy, InputData partner)
    {
        if (Utility.IsCollisionImminent(x, y, vx, vy, partner.X, partner.Y, partner.Vx, partner.Vy, 1150))
        { 
            return (oX, oY, (0).ToString());
        }

        (int pX, int pY) = (partner.X, partner.Y);

        double intendedTravelAngle = Math.Atan2(oY - y, oX - x);
        (int aX, int aY) = (vx + (int)Math.Floor(BURN_HARD * Math.Cos(intendedTravelAngle)), vy + (int)Math.Floor(BURN_HARD * Math.Sin(intendedTravelAngle)));

        (int fVX, int fVY) = (vx , vy);

        (int fX, int fY) = (x , y);
        for (int i = 0; i < 5; i++)
        {
            fVX += aX;
            fVY += aY;
            pX += partner.Vx;
            pY += partner.Vy;
            fX += fVX;
            fY += fVY;

            if(Utility.IsCollisionImminent(fX, fY, fVX, fVY, pX, pY, partner.Vx, partner.Vy, 1150))
            {
                if(partner.Vx >= 0)
                {
                    oX = pX - 1150 - 200;
                }
                else
                {
                    oX = pX + 1150 + 200;
                }

                if(partner.Vy >= 0)
                {
                    oY = pY - 1150 - 200;
                }
                else
                {
                    oY = pY + 1150 + 200;
                }

                Console.Error.WriteLine(" *** -- Evading Partner -- *** ");
                return (oX, oY, (50).ToString());

            }
        }

        return (oX, oY, (BURN_HARD).ToString());
    }


    private (int x, int y, string thrust) DetermineOutputsForTackle(int x, int y, int vx, int vy, int currentAngle, InputData targetData, int lapsCompleted)
    {
        (int projectedVx, int projectedVy) = this.AccountForTargetVelocity(x, y, vx, vy, targetData);
        (int destinationX , int destinationY) = this.AccountForTargetPositionWithExpectedTargetVelocity(x, y, vx, vy, targetData);

        int facingAngle = Utility.DetermineRotationalAngleToPoint(x, y, destinationX, destinationY, currentAngle);
        int distanceToTarget = this.ComputeDistanceToTarget(x, y, targetData);



        Utility.LocationDelegate fallbackDelegate = () => (destinationX, destinationY);
        (int oX, int oY) = this.DetermineCorrectedTargetPoint(x, y, destinationX, destinationY, vx, vy, distanceToTarget, facingAngle, fallbackDelegate, 0, DetermineCompensationFactor(vx, vy, targetData));

        if(Utility.IsCollisionImminent(x, y, vx, vy, targetData.X, targetData.Y, projectedVx, projectedVy))
        {
            return (oX, oY, "SHIELD");
        }

        return (oX, oY, (distanceToTarget < 500 ? BURN_STEADY : BURN_HARD).ToString());
    }

    private (int x, int y, string thrust) DetermineOutputsForZoneBlock(int x, int y, int vx, int vy, int currentAngle, InputData[] inputs, int lapsCompleted)
    {
        InputData targetData = inputs[this.currentTargetId];
        InputData alternateTarget = inputs[this.currentTargetId == 2 ? 3 : 2];

        int distanceToTarget = this.ComputeDistanceToTarget(x, y, targetData);
        if(distanceToTarget > (this.CheckPoints[this.relocationCheckPointId].LegDistance > 5000 ? 7000 : 5000))
        {
            return (targetData.X, targetData.Y, (0).ToString());
        }

        (int projectedVx, int projectedVy) = this.AccountForTargetVelocity(x, y, vx, vy, targetData);
        (int destinationX , int destinationY) 
            = this.AccountForCheckPointSkew(
                    this.AccountForTargetPositionWithExpectedTargetVelocity(x, y, vx, vy, targetData), 
                    projectedVx, 
                    projectedVy, 
                    this.CheckPoints[this.relocationCheckPointId].CheckpointX,
                    this.CheckPoints[this.relocationCheckPointId].CheckpointY,
                    200);

        int offSetForIdealFacingAngle = Utility.DetermineRotationalAngleToPoint(x, y, destinationX, destinationY, currentAngle);
            
        Utility.LocationDelegate fallbackDelegate = () => (destinationX, destinationY);
        (int oX, int oY) = this.DetermineCorrectedTargetPoint(x, y, destinationX, destinationY, vx, vy, distanceToTarget, offSetForIdealFacingAngle, fallbackDelegate, 0, DetermineCompensationFactor(vx, vy, targetData));



        if(Utility.IsCollisionImminent(x, y, vx, vy, targetData.X, targetData.Y, projectedVx, projectedVy))
        {
            return (oX, oY, "SHIELD");
        }

        (int projectedAltVx, int projectedAltVy) = this.AccountForTargetVelocity(x, y, vx, vy, alternateTarget);
        if(Utility.IsCollisionImminent(x, y, vx, vy, alternateTarget.X, alternateTarget.Y, projectedAltVx, projectedAltVy))
        {
            return (oX, oY, "SHIELD");
        }


        if(lapsCompleted > 0 && Utility.IsCollisionImminent(x, y, 
            vx + (int)Math.Floor(650 * Math.Cos((currentAngle + offSetForIdealFacingAngle).ToRadian())), 
            (int)Math.Floor(vy + 650 * Math.Sin((currentAngle + offSetForIdealFacingAngle).ToRadian())), 
            targetData.X, 
            targetData.Y, 
            projectedVx, 
            projectedVy,
            800))
        {
            return  (oX, oY, "BOOST");
        }
            
        bool needToRotate = Math.Abs(offSetForIdealFacingAngle) < 26 ? false : true;
        Console.Error.WriteLine($"idealFacingAngle {offSetForIdealFacingAngle} | currentAngle {currentAngle}");

        return (oX, oY, (needToRotate ? BURN_SLOW : BURN_HARD).ToString());
    }

    private double DetermineCompensationFactor(int vx, int vy, InputData targetData)
    {
        double compensationFactor = 1;
        double signAwareX = (vx * targetData.Vx);            
            
        if(signAwareX > 0)
        {
            compensationFactor += .05;
        }
        else if(signAwareX < 0)
        {
            compensationFactor -= .1;
        }         

        double signAwareY = (vy * targetData.Vy);            
         if(signAwareY > 0)
        {
            compensationFactor += .05;
        }
        else if(signAwareY < 0)
        {
            compensationFactor -= .1;
        }

        return compensationFactor;
    }

    private (int Vx, int Vy) AccountForTargetVelocity(int x, int y, int vx, int vy, InputData targetData, double maxDiffOffset = 1)
    {
        int projectedVx = targetData.Vx + (int)Math.Floor(Math.Cos(targetData.Angle) * BURN_HARD);
        int projectedVy = targetData.Vy + (int)Math.Floor(Math.Sin(targetData.Angle) * BURN_HARD);

        return (projectedVx, projectedVy);
    }

    private (int x, int y) AccountForTargetPositionWithExpectedTargetVelocity(int x, int y, int projectedVx, int projectedVy, InputData targetData)
    {
        int resultantX = targetData.X + projectedVx;
        int resultantY = targetData.Y + projectedVy;

        return (resultantX, resultantY);
    }

    private (int x, int y) AccountForCheckPointSkew((int X, int Y) target, int projectedVx, int projectedVy, int cpX, int cpY, double skewLength)
    {
        return Utility.ProjectDistanceFromPointToPoint(target.X, target.Y, cpX, cpY, skewLength);
    }

    #endregion

    // cache friendly distance computation
    private int ComputeDistanceToTarget(int x, int y, InputData targetData)
    {
        if(this.distanceToTarget == -1)
        {
            this.distanceToTarget = Utility.CalculateDistanceAsInt(x, y, targetData.X, targetData.Y);
        }
        return this.distanceToTarget;
    }
    
    private void ClearDataForEndCompute()
    {
        Console.Error.WriteLine($"{this.currentInterferenceStrategy.ToString("G")} Distance: {this.distanceToTarget}");
        this.distanceToTarget = -1;
        
    }
}

public class NodeLookup {
    public int AbsoluteAngleToCheckPoint { get; set; }
    public int ApproachY { get; set;}
    public int ApproachX { get; set;}
    public int BurnFactor { get; set;}
    public int BurnOffset { get; set;}
    public int CheckPointArrivalDeltaAngle { get; set; }
    public int CheckpointX { get; set;}
    public int CheckpointY { get; set;}
    public int InteferenceX { get; set;}
    public int InteferenceY { get; set;}
    public int LegDistance { get; set; }
    public int MidDistance { get; set; }
    public int NearDistance { get; set; }
    public int NextX { get; set;}
    public int NextY { get; set;}

    public void LogNodeToError() {
        Console.Error.WriteLine($"Approach ({this.ApproachX},{this.ApproachY}) | Dest {this.CheckpointX} {this.CheckpointY} | Interf ({this.InteferenceX},{this.InteferenceY})");
        Console.Error.WriteLine($"  AbsoluteAngleToCheckPoint {this.AbsoluteAngleToCheckPoint} | CheckPointArrivalDeltaAngle {this.CheckPointArrivalDeltaAngle}");
        Console.Error.WriteLine($"  Mid Distance{this.MidDistance} | Near Distance {this.NearDistance} | BurnFactor {this.BurnFactor} | LegDistance {this.LegDistance}");
    }
}

public struct InputData
{
    public int X;
    public int Y;
    public int Vx;
    public int Vy;
    public int Angle;
    public int NextCheckPointId;
}

public struct RacerData
{
    public int currentCheckpointId;
    public int checkPointCount;
}

public static class Utility
{
    public delegate (int x, int y) LocationDelegate();

    public static Double CalculateDistance(int x, int y, int x2, int y2) => Math.Sqrt( Math.Pow(x2-x, 2) + Math.Pow(y2-y, 2));

    public static int CalculateDistanceAsInt(int x, int y, int x2, int y2) => (int)Math.Floor(Utility.CalculateDistance(x, y ,x2, y2));

    public static int CalculateMinimum180AngleBetweenTwo360Angles(int firstAngle, int secondAngle) => (secondAngle - firstAngle).NormalizeAngleTo180(); 

    public static bool IsCollisionBad(int jx, int jy, int jvx, int jvy, int kx, int ky, int kvx, int kvy, int tx, int ty)
    {
        int dx = tx - jx;
        int dy = ty - jy;

        // if signs are in alignment with where we need to go.
        if(dx * jvx >= 0 && dy * jvy >= 0)
        {
            // If the enemy is also mostly in aligment this should push us or be minimally impactful which is generally good
            if(dx * kvx >= 0 || dx * kvy >= 0)
            {
                return false;
            }
        }

        if(dx * jvx <= 0 && dx * kvx >= 0)
        {
            // This should push us in an ideal situation, so it's good;
            return false;
        }

        if(dy * jvy <= 0 && dy * kvy >= 0)
        {
            // This should push us in an ideal situation, so it's good;
            return false;
        }
        return true;
    }

    public static bool IsCollisionImminent(int jx, int jy, int jvx, int jvy, int kx, int ky, int kvx, int kvy, int collisionDistance = 910)
    {
        int jXPost = jx + jvx;
        int jYPost = jy + jvy;
        int kXPost = kx + kvx;
        int kYPost = ky + kvy;

        int distance = CalculateDistanceAsInt(jXPost, jYPost, kXPost, kYPost);

        //Console.Error.WriteLine($"({jx},{jy}) + ({jvx},{jvy}) => ({jXPost},{jYPost})");
        //Console.Error.WriteLine($"({kx},{ky}) + ({kvx},{kvy}) => ({kXPost},{kYPost})");
        //Console.Error.WriteLine($"Distance {distance} collision {(distance <= 850 ? "Imminent" : "Not Imminent")}");

        return distance <= collisionDistance;
    }

    public static int NormalizeAngleTo360(this int angle) => (angle + 360) % 360;

    public static int NormalizeAngleTo180(this int angle) 
    {
        int threeSixtyAngle = NormalizeAngleTo360(angle);
        return threeSixtyAngle > 180 ? Math.Abs(threeSixtyAngle - 360) : threeSixtyAngle;
    }

    public static (int x, int y) GetShortVectorForRotateFromDegreesAngle(int x, int y, int angle)
    {
        double xAbsorb = Math.Cos(angle.ToRadian());
        double yAbsorb = Math.Sin(angle.ToRadian());

        return (x + (int)Math.Floor(100 * xAbsorb), y + (int)Math.Floor(100 * yAbsorb));

    }

    public static int GetSignedComplementOfAngle(int angle) => angle < 0 ? (-180 - angle) : (180 - angle);
    public static int ToDegrees(this double val) => (int)Math.Floor(val * (180/Math.PI));
    public static double ToRadian(this int val) => (double)val / 180 * Math.PI;

    public static int DetermineRotationalAngleToPoint(int x, int y, int targetX, int targetY, int currentFacingAngle) => Math.Abs(currentFacingAngle - DeterminePositionAngleToPoint(x, y, targetX, targetY).NormalizeAngleTo360()).NormalizeAngleTo180(); 
    public static int DeterminePositionAngleToPoint(int x, int y, int targetX, int targetY) => Math.Atan2(targetY - y, targetX - x).ToDegrees();

    public static int DetermineRelocationCheckpointForIntercept(Dictionary<int, NodeLookup> checkPoints, int x, int y, InputData targetData)
    {
        // Initial comparison. Only allow relocation if significantly shorter traversal distance
        int interceptCost = BallparkTraversalValue(x, y, checkPoints[targetData.NextCheckPointId].CheckpointX, checkPoints[targetData.NextCheckPointId].CheckpointY);
        int targetCost = BallparkTraversalValue(targetData.X, targetData.Y, checkPoints[targetData.NextCheckPointId].CheckpointX, checkPoints[targetData.NextCheckPointId].CheckpointY);

        if(targetCost - interceptCost > 1000) 
        {
            //Console.Error.WriteLine($"CheckPoint Determination Completed {targetData.NextCheckPointId} {targetCost} - {interceptCost} > 1000");
            return targetData.NextCheckPointId;
        }

        //Console.Error.WriteLine($"CheckPoint Determination Continues {targetData.NextCheckPointId} {targetCost} - {interceptCost} <= 1000");

        int currentCheckPointForInspection = targetData.NextCheckPointId;
        do
        {
            int priorCheckPoint = currentCheckPointForInspection;
            currentCheckPointForInspection = currentCheckPointForInspection == checkPoints.Count - 1 ? 0 : currentCheckPointForInspection + 1;

            interceptCost = BallparkTraversalValue(x, y, checkPoints[currentCheckPointForInspection].CheckpointX, checkPoints[currentCheckPointForInspection].CheckpointY);                
            targetCost += BallparkTraversalValue(checkPoints[priorCheckPoint].CheckpointX, checkPoints[priorCheckPoint].CheckpointY, checkPoints[currentCheckPointForInspection].CheckpointX, checkPoints[currentCheckPointForInspection].CheckpointY);

            if(targetCost - interceptCost > 500)
            {
                //Console.Error.WriteLine($"CheckPoint Determination {currentCheckPointForInspection} {targetCost} - {interceptCost} > 500");
                return currentCheckPointForInspection;
            }
            //Console.Error.WriteLine($"CheckPoint Determination Continues {targetData.NextCheckPointId} {targetCost} - {interceptCost} <= 500");
        }
        while(targetData.NextCheckPointId != currentCheckPointForInspection);

        //Console.Error.WriteLine($"CheckPoint Determination all choices were bad");
        return targetData.NextCheckPointId == checkPoints.Count - 1 ? 0 : targetData.NextCheckPointId + 1;
    }

    public static int BallparkTraversalValue(int x, int y, int tx, int ty)
    {
        return CalculateDistanceAsInt(x, y, tx, ty);
    }

    public static (int x, int y, int burnOffset) DetermineApproachPosition(NodeLookup previousNode, NodeLookup node, int leadDistance)
    {

        int approachAngleAbsolutePosition = (previousNode.AbsoluteAngleToCheckPoint - node.CheckPointArrivalDeltaAngle).NormalizeAngleTo360();

                
        int angleIncrements = (int)Math.Floor((double)(180 - Math.Abs(node.CheckPointArrivalDeltaAngle)) / 5);

        int burnOffset = angleIncrements * 100; //Math.Max((int)Math.Floor(leadDistance * ((double)node.LegDistance / 3000)), 1000);

        double xAbsorb = Math.Cos(approachAngleAbsolutePosition.ToRadian());
        double yAbsorb = Math.Sin(approachAngleAbsolutePosition.ToRadian());

        int approachX = node.CheckpointX + (int)Math.Floor(xAbsorb * burnOffset);
        int approachY = node.CheckpointY + (int)Math.Floor(yAbsorb * burnOffset);
        
         Console.Error.WriteLine($"prevAbs {previousNode.AbsoluteAngleToCheckPoint} | delta {node.CheckPointArrivalDeltaAngle} |  burnOffset {burnOffset}");
         Console.Error.WriteLine($"Previous ({previousNode.CheckpointX},{previousNode.CheckpointY}) |  Current : ({node.CheckpointX},{node.CheckpointY}) | base ({node.CheckpointX},{node.CheckpointY}) =>   Approach : ({approachX},{approachY})");

        return (approachX, approachY, burnOffset);

    }

    public static (int x, int y, int burnOffset) DetermineApproachPositionPrevious(NodeLookup previousNode, NodeLookup node, int leadDistance)
    {
        int baseX = node.CheckpointX;//(int) Math.Floor((node.CheckpointX + previousNode.CheckpointX) * .5);
        int baseY = node.CheckpointY;//(int) Math.Floor((node.CheckpointY + previousNode.CheckpointY) * .5);

        bool offsetClockWise = node.CheckPointArrivalDeltaAngle < 0 || node.CheckPointArrivalDeltaAngle > 180;

        int offsetAngle = (previousNode.AbsoluteAngleToCheckPoint + (offsetClockWise ? 90 : -90)).NormalizeAngleTo360();
                
        int angleIncrements = (int)Math.Floor((double)(180 - Math.Abs(node.CheckPointArrivalDeltaAngle)) / 5);

        int burnOffset = angleIncrements * 100; //Math.Max((int)Math.Floor(leadDistance * ((double)node.LegDistance / 3000)), 1000);

        double xAbsorb = Math.Cos(offsetAngle.ToRadian());
        double yAbsorb = Math.Sin(offsetAngle.ToRadian());

        int approachX = baseX + (int)Math.Floor(xAbsorb * burnOffset);
        int approachY = baseY + (int)Math.Floor(yAbsorb * burnOffset);
        
         Console.Error.WriteLine($"prevAbs {previousNode.AbsoluteAngleToCheckPoint} | delta {node.CheckPointArrivalDeltaAngle} |  burnOffset {burnOffset}");
         Console.Error.WriteLine($"offsetClockWise {offsetClockWise} | offsetAngle: {offsetAngle} | xAbsorb {xAbsorb} | yAbsorb {yAbsorb} ");
         Console.Error.WriteLine($"Previous ({previousNode.CheckpointX},{previousNode.CheckpointY}) |  Current : ({node.CheckpointX},{node.CheckpointY}) | base ({baseX},{baseY}) =>   Approach : ({approachX},{approachY})");

        return (approachX, approachY, burnOffset);

    }

    public static (int x, int y) DetermineInterferenceHoldingPosition(NodeLookup previousNode, NodeLookup node)
    {
        int angleToPreviousCheckPoint = (previousNode.AbsoluteAngleToCheckPoint - 180).NormalizeAngleTo360();

        double radianAngleToInterferencePoint = (angleToPreviousCheckPoint +  node.CheckPointArrivalDeltaAngle >= 0 ? -90 : 90).ToRadian();

        return (node.CheckpointX + (int)Math.Floor(Math.Cos(radianAngleToInterferencePoint) * 750), node.CheckpointY + (int)Math.Floor(Math.Sin(radianAngleToInterferencePoint) * 750));
    }


    public static bool CurrentVelocityWillHitAndOvershootTarget(int x, int y, int vx, int vy, int xTarget, int yTarget, double overShootThreshold, double hitThreshHold = 500)
    {
        double finalX = x;
        double finalY = y;

        double currentVx = vx;
        double currentVy = vy;

        bool targetHit = false;
        //Console.Error.WriteLine($"Begin Projection");

        while(Math.Abs(currentVx) > 5 || Math.Abs(currentVy) > 5)
        {
            finalX = finalX + currentVx;
            currentVx = currentVx *.85;

            finalY = finalY + currentVy;
            currentVy = currentVy *.85;

            if(!targetHit)
            {
                double distanceAtThisPointToGoal = Utility.CalculateDistance((int)Math.Floor(finalX), (int)Math.Floor(finalY), xTarget, yTarget);
                if(distanceAtThisPointToGoal < 500)
                {
                    targetHit = true;
                }
            }
            //Console.Error.WriteLine($"({finalX},{finalY})");
            
        }
        //Console.Error.WriteLine($"End Projection");

        

        double overshootDistance = Utility.CalculateDistance((int)Math.Floor(finalX), (int)Math.Floor(finalY), xTarget, yTarget); 

        //Console.Error.WriteLine($"Will {(targetHit ? "hit" : "miss")} target | overshoot {overshootDistance} | ({finalX},{finalY}) ");

        return targetHit && overshootDistance > overShootThreshold;
    }

    public static (int x, int y) ProjectDistanceFromPointToPoint(int x, int y, int tX, int tY, double skewLength)
    {
        double angleToCheckPoint = Math.Atan2(tY - y, tX - x );

        double projectedX = x + (skewLength * Math.Cos(angleToCheckPoint));
        double projectedY = y + (skewLength * Math.Sin(angleToCheckPoint));

        return ( (int)Math.Floor(projectedX), (int)Math.Floor(projectedY) );
    }
    

    public static Dictionary<int, NodeLookup> NodeBuild((int x, int y)[] checkPoints)
    {
        Dictionary<int, NodeLookup> returnCheckPoints = new Dictionary<int, NodeLookup>();
        for (int i = 0; i < checkPoints.Length; i++)
        {
            returnCheckPoints[i] = new NodeLookup 
        {
        CheckpointX = checkPoints[i].x,
        CheckpointY = checkPoints[i].y,
        NextX = checkPoints[ (i < checkPoints.Length - 1) ? i+1 : 0].x,
        NextY = checkPoints[ (i < checkPoints.Length - 1) ? i+1 : 0].y
        };

        returnCheckPoints[i].AbsoluteAngleToCheckPoint = Math.Atan2(returnCheckPoints[i].NextY - returnCheckPoints[i].CheckpointY, returnCheckPoints[i].NextX - returnCheckPoints[i].CheckpointX).ToDegrees().NormalizeAngleTo360();
        returnCheckPoints[i].BurnFactor =
            (returnCheckPoints[i].CheckPointArrivalDeltaAngle > 155 && returnCheckPoints[i].CheckPointArrivalDeltaAngle < 205)
                ? Racing.BURN_STALL
                : Racing.BURN_SLOW;
        returnCheckPoints[i].LegDistance = Utility.CalculateDistanceAsInt(returnCheckPoints[i].CheckpointX, returnCheckPoints[i].CheckpointY, returnCheckPoints[i].NextX, returnCheckPoints[i].NextY);
        } 

        for(int i = 0; i < checkPoints.Length; i++) 
        {
            NodeLookup workingNode = returnCheckPoints[i];
            NodeLookup previousNode = returnCheckPoints[i == 0 ? checkPoints.Length-1 : i-1];
            workingNode.CheckPointArrivalDeltaAngle = (workingNode.AbsoluteAngleToCheckPoint - previousNode.AbsoluteAngleToCheckPoint).NormalizeAngleTo360();
            workingNode.MidDistance = Math.Max((workingNode.LegDistance / 500) * 200, 2000);
            workingNode.NearDistance = Math.Max((workingNode.LegDistance / 500) * 50, (returnCheckPoints[i].CheckPointArrivalDeltaAngle > 155 && returnCheckPoints[i].CheckPointArrivalDeltaAngle < 205) ? 2000 : 500);
            (workingNode.ApproachX, workingNode.ApproachY, workingNode.BurnOffset) = Utility.DetermineApproachPosition(previousNode, workingNode, Racing.APPROACH_DISTANCE);
            (workingNode.InteferenceX, workingNode.InteferenceY) = Utility.DetermineInterferenceHoldingPosition(previousNode, workingNode);
        }
        for(int i = 0; i < checkPoints.Length; i++) 
        {
            returnCheckPoints[i].LogNodeToError();
        }
        return returnCheckPoints;
    }

}