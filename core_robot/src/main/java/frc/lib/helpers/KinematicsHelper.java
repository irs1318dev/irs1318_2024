package frc.lib.helpers;

/**
 * Class to contain the functions describing the velocity and position of the projectile.
 * Some fields are commented out and will be un-commented if deemed necessary
 * @author made by Nishant
 */
public class KinematicsHelper 
{
//===========Values for drag calculations=======
    
    //private double stokesFlowDragCoefficient;
    //private double compoundStokesDragForceCoefficient;

    private double newtonianDragCoefficient;
    private double crossSectionalArea;
    private double fluidDensity;
    private double compoundNewtonianDragForceCoefficient;
    private double dragForce;
    private double fluidViscosity;
//=======Initial conditions=====================
    private double initialVelocity;

    private double initialPositionY;
    private double initialPositionX;
    private double initialAngle;
//===========Constants===============
    private static final double gravitationalConstantInMeters = 9.81;
    private static double degreesToRadians = Math.PI / 180;
//============Other Values===========================================
    private double objectMass;
    private UnitHandler unitsOfSystem;
//========Unit Manipulation Enumeration=====
    private enum UnitHandler 
    {
        Meters(39.37008),
        Inches(1),
        Feet(12);

        private double conversionFactorToInches;

        private UnitHandler(double unit)
        {
            this.conversionFactorToInches = unit;
        }

        public static double convert(UnitHandler originalUnitType, UnitHandler targetUnitType, double value)
        {
            double conversionFactor = originalUnitType.conversionFactorToInches / targetUnitType.conversionFactorToInches;
            return conversionFactor * value;
        }
    }
//==========Constructors=========================================
    /**
     * This is the base constructor
     * @param newtonianDragCoefficient A unique valued that needs to be determined experimentally
     * @param mass The mass of the torus
     * @param initialVelocity The initial velocity the torus is being launched at
     * @param units The units of choice for the handler
     * @param intialAngle The angle at which the torus is launched
     * @param fluidDensity The density of the fluid it is passing through
     * @param initialPositionX The initial x-coordinate of the projectile
     * @param initialPositionY The initial y-coordinate of the projectile
     * 
     */
    public KinematicsHelper(
    /*double stokesFlowDragCoefficient,*/ 
        double newtonianDragCoefficient,
        double mass,
        double initialVelocity,
        UnitHandler units,
        double intialAngle,
        double fluidDensity,
        double initialPositionX,
        double initialPositionY)
    {
        //this.stokesFlowDragCoefficient = stokesFlowDragCoefficient;
        this.objectMass = mass;
        this.initialVelocity = initialVelocity;
        //this.compoundstokesDragForceCoefficient = stokesFlowDragCoefficient / objectMass;
        this.unitsOfSystem = units;
        this.initialAngle = intialAngle;
        this.fluidDensity = fluidDensity;
        this.compoundNewtonianDragForceCoefficient = 0.5 * fluidDensity * newtonianDragCoefficient * crossSectionalArea;
        this.initialPositionX = initialPositionX;
        this.initialPositionY = initialPositionY;
    }

    /**
     * This is an extra constructor for if fluid viscosity is ever needed
     * @param newtonianDragCoefficient A unique valued that needs to be determined experimentally
     * @param mass The mass of the torus
     * @param initialVelocity The initial velocity the torus is being launched at
     * @param units The units of choice for the handler
     * @param intialAngle The angle at which the torus is launched
     * @param fluidDensity The density of the fluid it is passing through
     * @param initialPositionX The initial x-coordinate of the projectile
     * @param initialPositionY The initial y-coordinate of the projectile
     * @param fluidViscosity The vicosity of the fluid, needed only for Stokes flow resistance
     */
    public KinematicsHelper(
        /*double stokesDragCoefficient, */
        double newtonianDragCoefficient,
        double mass,
        double initialVelocity,
        UnitHandler units,
        double intialAngle,
        double fluidDensity,
        double initialPositionX,
        double initialPositionY,
        double fluidViscosity)

    {
        this(/*dragCoefficient,*/ newtonianDragCoefficient, mass, initialVelocity, units, intialAngle, fluidDensity, initialPositionX, initialPositionY);
        this.fluidViscosity = fluidViscosity;

    }
//==============   Methods   =====================================================================================================================================
    /**
     * This generates the velocity vector at whatever time is passed as input in seconds
     * @param time time at which function is being evaluated, in units of seconds
     * @param units the choice of units for the output
     * @return the velocity as a vector with x and y components
     */
    public double[] getVelocityVectorFromNewtonianDrag(double time, UnitHandler units)
    {
        double[] newtonianDragVelocityVector = {
            UnitHandler.convert(UnitHandler.Meters, units, newtonianDragBasedVelocityXComponent(time)),
            UnitHandler.convert(UnitHandler.Meters, units, newtonianDragBasedVelocityYComponent(time))
        };
        return newtonianDragVelocityVector;
    }

    /**
     * All evaluations done in SI Units
     * @param time time at which function is being evaluated, in seconds
     * @return the x-component of the velocity using newtonian drag
     */
    private double newtonianDragBasedVelocityXComponent(double time) {
        double initialVelocityX = this.initialVelocity * Math.cos(this.initialAngle * degreesToRadians);
        double velocityX = (initialVelocityX) / (1 - this.compoundNewtonianDragForceCoefficient * initialVelocityX * time);
        return velocityX;
    }

    /**
     * All evaluations done in SI Units
     * @param time time at which function is being evaluated, in seconds
     * @return returns the y-component of the velocity using newtonian drag
     */
    private double newtonianDragBasedVelocityYComponent(double time)
    {
        double initialVelocityY = this.initialVelocity * Math.sin(this.initialAngle * degreesToRadians);
        double rootGravitationalCompoundOverDragForceCoefficient = Math.sqrt(gravitationalConstantInMeters / this.compoundNewtonianDragForceCoefficient);
        double scaledArctanOfInitialVelocity = Math.atan(initialVelocityY / rootGravitationalCompoundOverDragForceCoefficient);
        double velocityY = rootGravitationalCompoundOverDragForceCoefficient * Math.tan(time / rootGravitationalCompoundOverDragForceCoefficient + scaledArctanOfInitialVelocity);
        return velocityY;
    }

    /**
     * @param time time at evaluation
     * @param units choice of units
     * @return the position as a vector
     */
    public double[] getPositionVectorFromNewtonianDrag(double time, UnitHandler units)
    {
        double[] position = {
            UnitHandler.convert(UnitHandler.Meters, units, newtonianDragBasedPositionXComponent(time)),
            UnitHandler.convert(UnitHandler.Meters, units, newtonianDragBasedPositionYComponent(time))};
        return position;
    }

    /**
     * @param time time at evauluation of position
     * @return the y-component of the position
     */
    private double newtonianDragBasedPositionYComponent(double time) 
    {
        double rootGravitationalCompoundOverDragForceCoefficient = Math.sqrt(gravitationalConstantInMeters / this.compoundNewtonianDragForceCoefficient);
        
        double initialVelocityY = this.initialVelocity * Math.sin(this.initialAngle * degreesToRadians);
        double scaledArctanOfInitialVelocity = Math.atan(initialVelocityY / rootGravitationalCompoundOverDragForceCoefficient);
        
        double positionY = this.initialPositionY - Math.pow(rootGravitationalCompoundOverDragForceCoefficient,2) * Math.log(
            Math.abs(Math.cos(time / rootGravitationalCompoundOverDragForceCoefficient + scaledArctanOfInitialVelocity))
        );
        return positionY;
    }

    /**
     * @param time time at evaluation
     * @return the x-component of position
     */
    private double newtonianDragBasedPositionXComponent(double time) 
    {
        double initialVelocityX = this.initialVelocity * Math.cos(this.initialAngle * degreesToRadians);
        double positionX = this.initialPositionX + 
        (Math.log(Math.abs(1 - this.compoundNewtonianDragForceCoefficient * initialVelocityX * time))) / this.compoundNewtonianDragForceCoefficient;
        
        return positionX;
    }
}
