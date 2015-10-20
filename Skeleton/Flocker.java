import java.awt.Color;
import java.awt.Graphics;
import java.io.BufferedWriter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

import org.xml.sax.Attributes;
import org.xml.sax.Locator;
import org.xml.sax.SAXException;

/**
 * An agent implementing a basic version of Reynold's
 * flocking algorithm from SIGGRAPH 1987.
 * Designed to provide skeleton code.
 * 
 * @version 1.1
 */
public class Flocker extends Follower {

    /** Element name for entities of this type in XML documents */
    static final String XML_NAME = "flocker";

    /**
     * Distinctive static properties of a flocker
     */
    static class FlockerAttributes {

    	/** XML attribute for setting obstacle avoidance */
    	static final String AVOIDS_OBSTACLES_PARAM = "clear";
    	
        /** XML attribute for setting collision avoidance */
        static final String AVOIDS_COLLISIONS_PARAM = "evade";

        /** XML attribute for setting alignment with neighbors */
        static final String ALIGNS_WITH_NEIGHBORS_PARAM = "align";

        /** XML attribute for setting collision avoidance */
        static final String DOES_CENTERING_PARAM = "center";
        
        /** XML attribute for setting food following */
        static final String FOLLOWS_LIGHT_PARAM = "follow";

        /** XML attribute for distance at which obstacles are a worry */
        static final String CLEARANCE_DISTANCE_PARAM = "clearance";
        
        /** XML attribute for angle at which obstacles are a worry */
        static final String CLEARANCE_ANGLE_PARAM = "cone";
        
        /** XML attribute for setting minimum separation distance */
        static final String SEPARATION_DISTANCE_PARAM = "separation";

        /** XML attribute for setting distance at which agent starts to pay attention */
        static final String DETECTION_DISTANCE_PARAM = "detection";

        /** XML attribute for setting obstacle weight */	
        static final String OBSTACLE_WEIGHT_PARAM = "ow";
        
        /** XML attribute for setting separation weight */
        static final String SEP_WEIGHT_PARAM = "sw";

        /** XML attribute for setting alignment weight */
        static final String ALIGN_WEIGHT_PARAM = "aw";

        /** XML attribute for setting centering weight */
        static final String CTR_WEIGHT_PARAM = "cw";
        
        /** XML attribute for setting importance of following light */
        static final String LIGHT_WEIGHT_PARAM = "lw";

        /** Does the flocker avoid obstacles */
        boolean avoidsObstacles;
        /** How close must an obstacle be before collision is a worry */
        double clearance;
        /** How close ahead must an obstacle be before collision is a worry */
        double cone;
        /** How important is avoiding obstacles */
        double obstacleWeight;

        
        /** Does the flocker avoid collisions */
        boolean avoidsCollisions;
        /** How close must a neighbor be before collision is a worry */
        double separationDistance;
        /** How important is avoiding collision compared to other goals */
        double separationWeight;


        /** How close must other things be to affect boid's decisions */
        double detectionDistance;
        /** Does the boid try to fly in the same direction as neighbors? */
        boolean alignsWithNeighbors;
        /** How important is aligning with neighbors compared to other goals? */
        double alignmentWeight;


        /** Does the boid try to fly in the middle of its local group? */
        boolean doesCentering;
        /** How important is flying in the center compared to other goals? */
        double centeringWeight;


        /** Does the bird follow (and eat) light sources */
        boolean followsLight;
        /** How important is following light compared to other goals? */
        double followWeight;
        



        /**
         * Constructor from specific data
         * 
         * @param obstacles true if boid should avoid obstacles
         * @param collide true if boid should avoid collisions with neighbors
         * @param align true if boid should align heading with neighbors
         * @param center true if boid should stay in midst of neighbors
         * @param follow true if boid should follow lights
         * @param clearance distance threshold for obstacles
         * @param cone angle threshold for obstacles
         * @param separation distance threshold for evasive action
         * @param detection distance threshold for other action
         * @param ow strength of obstacle drive
         * @param sw strength of separation drive
         * @param aw strength of alignment drive
         * @param cw strength of centering drive
         * @param fw strength of light drive
         */
        FlockerAttributes(boolean obstacles, boolean collide, 
        		boolean align, boolean center, boolean follow,
                double clearance, double cone,
                double separation, double detection,
                double ow, double sw, double aw, double cw, double fw) {
            avoidsObstacles = obstacles;
            avoidsCollisions = collide;
            alignsWithNeighbors = align;
            doesCentering = center;
            followsLight = follow;
            this.clearance = clearance;
            this.cone = cone;
            separationDistance = separation;
            detectionDistance = detection;
            this.obstacleWeight = ow;
            separationWeight = sw;
            alignmentWeight = aw;
            centeringWeight = cw;
            followWeight = fw;
        }

        /**
         * Constructor from XML data
         * 
         * @param atts SAX attributes corresponding to XML data
         * @param defaults values to use when data is unspecified
         * @param locator file information for error messages
         * @throws SAXException if data has wrong format
         */
        public void set(Attributes atts, FlockerAttributes defaults, Locator locator)
        throws SAXException {
            avoidsObstacles = FlockingReader.getBoolParam(atts, AVOIDS_OBSTACLES_PARAM, defaults.avoidsObstacles, locator);
            avoidsCollisions = FlockingReader.getBoolParam(atts, AVOIDS_COLLISIONS_PARAM, defaults.avoidsCollisions, locator);
            alignsWithNeighbors = FlockingReader.getBoolParam(atts, ALIGNS_WITH_NEIGHBORS_PARAM, defaults.alignsWithNeighbors, locator);
            doesCentering = FlockingReader.getBoolParam(atts, DOES_CENTERING_PARAM, defaults.doesCentering, locator);
            followsLight = FlockingReader.getBoolParam(atts, FOLLOWS_LIGHT_PARAM, defaults.followsLight, locator);
            clearance = FlockingReader.getDoubleParam(atts, CLEARANCE_DISTANCE_PARAM, defaults.clearance, locator);
            double degrees  = FlockingReader.getDoubleParam(atts, CLEARANCE_ANGLE_PARAM, defaults.cone * RADIANS_TO_DEGREES, locator);
          	cone = World.clampToCircle(degrees * DEGREES_TO_RADIANS, 2 * Math.PI);
            separationDistance = FlockingReader.getDoubleParam(atts, SEPARATION_DISTANCE_PARAM, defaults.separationDistance, locator);
            detectionDistance = FlockingReader.getDoubleParam(atts, DETECTION_DISTANCE_PARAM, defaults.detectionDistance, locator);
            obstacleWeight = FlockingReader.getDoubleParam(atts, OBSTACLE_WEIGHT_PARAM, defaults.obstacleWeight, locator);
            separationWeight = FlockingReader.getDoubleParam(atts, SEP_WEIGHT_PARAM, defaults.separationWeight, locator);
            alignmentWeight = FlockingReader.getDoubleParam(atts, ALIGN_WEIGHT_PARAM, defaults.alignmentWeight, locator);
            centeringWeight = FlockingReader.getDoubleParam(atts, CTR_WEIGHT_PARAM, defaults.centeringWeight, locator);
            followWeight = FlockingReader.getDoubleParam(atts, LIGHT_WEIGHT_PARAM, defaults.followWeight, locator);
        }

        /**
         * Change parameters based on XML data
         * 
         * @param atts SAX attributes corresponding to XML data
         * @param locator file information for error messages
         * @throws SAXException if data has wrong format
         */
        public void update(Attributes atts, Locator locator)
        throws SAXException {
        	set(atts, this, locator);
        }

        /**
         * Write a complete XML description of the flocking boid
         * (in its current state) on the file corresponding to out
         * 
         * @param out where data should be written
         * @throws IOException if writes fail
         */
        public void log(BufferedWriter out)
        throws IOException {
        	double degrees = RADIANS_TO_DEGREES * cone;
        	
            out.write(
                    AVOIDS_OBSTACLES_PARAM + OPEN + Boolean.toString(avoidsObstacles) + CLOSE +
                    AVOIDS_COLLISIONS_PARAM + OPEN + Boolean.toString(avoidsCollisions) + CLOSE +
                    ALIGNS_WITH_NEIGHBORS_PARAM + OPEN + Boolean.toString(alignsWithNeighbors) + CLOSE +
                    DOES_CENTERING_PARAM + OPEN + Boolean.toString(doesCentering) + CLOSE +
                    FOLLOWS_LIGHT_PARAM + OPEN + Boolean.toString(followsLight) + CLOSE +
                    CLEARANCE_DISTANCE_PARAM + OPEN + Double.toString(clearance) + CLOSE +
                    CLEARANCE_ANGLE_PARAM + OPEN + Double.toString(degrees) + CLOSE +
                    SEPARATION_DISTANCE_PARAM + OPEN + Double.toString(separationDistance) + CLOSE +
                    DETECTION_DISTANCE_PARAM + OPEN + Double.toString(detectionDistance) + CLOSE +
                    OBSTACLE_WEIGHT_PARAM + OPEN + Double.toString(obstacleWeight) + CLOSE +
                    SEP_WEIGHT_PARAM + OPEN + Double.toString(separationWeight) + CLOSE +
                    ALIGN_WEIGHT_PARAM + OPEN + Double.toString(alignmentWeight) + CLOSE +
                    CTR_WEIGHT_PARAM + OPEN + Double.toString(centeringWeight) + CLOSE +
                    LIGHT_WEIGHT_PARAM + OPEN + Double.toString(followWeight) + CLOSE +
                    "\n");

        }

        /**
         * Constructor from XML data
         * 
         * @param atts SAX attributes defining flocker element
         * @param defaults what to do when attributes are unspecified
         * @param loc file information for error messages
         * @throws SAXException if data has wrong format
         */
        FlockerAttributes(Attributes atts, FlockerAttributes defaults, Locator loc) 
        throws SAXException {
            set(atts, defaults, loc);
        }
    }

    /** Should a flocker by default avoid obstacles */
    static final boolean AVOIDS_OBSTACLES = true;
    
    /** Should a flocker by default avoid collisions */
    static final boolean AVOIDS_COLLISIONS = true;

    /** Should a flocker by default align with neighbors */
    static final boolean ALIGNS_WITH_NEIGHBORS = true;

    /** Should a flocker by default try to maneuver to the center of the flock */
    static boolean DOES_CENTERING = true;
    
    /** Should a flocker follow lights? */
    static boolean FOLLOWS_LIGHT = true;

    /** By default, how close can individuals in the flock get before they get antsy */
    static final int DEFAULT_SEPARATION_DISTANCE = 50;

    /** By default, how much of the world does an individual in the flock attend to */
    static int DEFAULT_DETECTION_DISTANCE = 250;

    /** By default, when does an agent get worried about obstacles */
    static final double DEFAULT_CLEARANCE_DISTANCE = 140;
    
    /** By default, what headings count as an approaching crash */
    static final double DEFAULT_CLEARANCE_ANGLE = 60 * DEGREES_TO_RADIANS;
    
    /** By default, how hard do agents work to avoid obstacles */
    static final double OBSTACLE_WEIGHT = 2.0;
    
    /** By default, how hard do agents work to avoid collisions */
    static final double SEPARATION_WEIGHT = 2.0;

    /** By default, how hard do agents work to align with one another */
    static double ALIGNMENT_WEIGHT = 5.0;

    /** By default, how hard do agents work at centering */
    static double CENTERING_WEIGHT = 10.0;
    
    /** By default, how hard to agents work to follow lights */
    static double LIGHT_WEIGHT = 5.0;

    /** Record that allows XML files to set Flocker defaults */
    static FlockerAttributes defaultFlockerAttributes =
        new FlockerAttributes(AVOIDS_OBSTACLES,
        		AVOIDS_COLLISIONS, ALIGNS_WITH_NEIGHBORS,
                DOES_CENTERING, FOLLOWS_LIGHT,
                DEFAULT_CLEARANCE_DISTANCE,
                DEFAULT_CLEARANCE_ANGLE,
                DEFAULT_SEPARATION_DISTANCE,
                DEFAULT_DETECTION_DISTANCE,
                OBSTACLE_WEIGHT, SEPARATION_WEIGHT, 
                ALIGNMENT_WEIGHT, CENTERING_WEIGHT, LIGHT_WEIGHT);

    /** Record that allows XML files to set Flocker defaults */
    static FixedAgentAttributes defaultFixedAgentAttributes =
        new Agent.FixedAgentAttributes(
                FOLLOWER_SIZE,
                FOLLOWER_MAX_SPEED,
                FOLLOWER_MIN_SPEED,
                FOLLOWER_MAX_ACCEL,
                FOLLOWER_MAX_DECEL,
                FOLLOWER_MAX_TURN,
                0, HALF_CIRCLE, new Color(150, 0, 150),
                false, false);

    /** Record that allows XML files to set Flocker defaults
    static DynamicAgentAttributes defaultDynamicAgentAttributes =
        new Agent.DynamicAgentAttributes(250, 250, 0, 0);

    /**
     * This is a convenience class that handles the
     * integration and visualization of control 
     * information from different sources
     */
    static class WeightedForce {
    	static final double PIXELS_PER_UNIT_WEIGHT = 8;
    	static final double ARROWHEAD_LENGTH = 10;
    	
    	/** need to change (forward or back) in direction of heading */
        public double fx;
        /** need to change (right or left) perpendicular to heading */
        public double fy;
        
        /** 
         * Constructor
         * @param x forward--backward force value
         * @param y right--left force value
         */
        public WeightedForce(double weight, double angle) {
            fx = weight * Math.cos(angle); 
            fy = weight * Math.sin(angle);
        }
        /**
         * Default constructor for zero force
         */
        public WeightedForce() {
        	fx = 0; fy = 0;
        }
        /**
         * @return overall strength of force
         */
        public double getWeight() {
            return Math.sqrt(fx * fx + fy * fy);
        }
        
        /**
         * @return direction of force relative to heading
         */
        public double getAngle() {
            return World.displacementOnCircle(0, Math.atan2(fy, fx), 2 * Math.PI);
        }
        
        /**
         * Update force to include new component
         * @param f additional force to take into account
         */
        public void addIn(WeightedForce f) {
        	fx += f.fx; fy += f.fy;
        }
        /**
         * Weight the force by a factor
         * @param factor multiplicative change to force
         */
        public void reweight(double factor) {
        	fx *= factor; fy *= factor;
        }
        
        /**
         * Draw on the screen
         * @param x center of drawing
         * @param y center of drawing
         * @param theta heading direction
         * @param world to control overlap
         * @param g for display attributes
         */
        public void draw(DynamicAgentAttributes a, World w, Graphics g) {
        	double segmentAngle = a.heading + getAngle();
        	double topArrowAngle = segmentAngle - 5 * Math.PI / 6;
        	double bottomArrowAngle = segmentAngle + 5 * Math.PI / 6;
        	double length = PIXELS_PER_UNIT_WEIGHT * getWeight();
        	double x1 = a.locX + length * Math.cos(segmentAngle);
        	double y1 = a.locY + length * Math.sin(segmentAngle);
        	w.drawLine((int)Math.round(a.locX), (int)Math.round(a.locY),
        			(int)Math.round(x1),(int)Math.round(y1), g);
        	w.drawLine((int)Math.round(x1), (int)Math.round(y1),
        			(int)Math.round(x1 + ARROWHEAD_LENGTH * Math.cos(topArrowAngle)),
        			(int)Math.round(y1 + ARROWHEAD_LENGTH * Math.sin(topArrowAngle)),
        			g);
           	w.drawLine((int)Math.round(x1), (int)Math.round(y1),
        			(int)Math.round(x1 + ARROWHEAD_LENGTH * Math.cos(bottomArrowAngle)),
        			(int)Math.round(y1 + ARROWHEAD_LENGTH * Math.sin(bottomArrowAngle)),
        			g);
        }
    }
    
    /** Decision making parameters for this boid's flocking behavior */
    FlockerAttributes flocking;

    /** Record of previous decisions for visualization */
    WeightedForce safety = null;
    WeightedForce collision = null;
    WeightedForce alignment = null;
    WeightedForce centering = null;
    WeightedForce light = null;
    WeightedForce total = null;
    
    /**
     * Constructor: initialize general agent fields to describe
     * a flocking agent.
     * 
     * @param w world to which agent belongs
     * @param id number to identify agent in its world
     * @param atts SAX attributes corresponding to XML agent spec
     * @param loc file information for error messages
     * @throws SAXException if data is formatted incorrectly
     */
    Flocker(World w, int id, Attributes atts, Locator loc)
    throws SAXException {
        super(w, id, atts, loc);
        form.set(atts, defaultFixedAgentAttributes, loc);
        status.set(atts, defaultDynamicAgentAttributes, loc);
        flocking = new FlockerAttributes(atts, defaultFlockerAttributes, loc);
  }

    /**
     * Output an XML element describing the current state of
     * this boid.
     * 
     * @param out an open file to write to, wrapped in BufferedWriter 
     *            convenience class
     */
    @Override
    public void log(BufferedWriter out) throws IOException {
        out.write("   <" + XML_NAME + " " + ID_PARAM + OPEN + Integer.toString(id) + CLOSE + "\n     ");
        form.log(out);
        out.write("     ");
        status.log(out);
        out.write("     ");
        flocking.log(out);
        out.write("    />\n");
    }

    /**
     * Specialized drawing method in case you want debugging help
     */
    @Override
    public void draw(Graphics g) {

        if (form.debug && lastStatus != null) {
           	form.debug = false;
           	super.draw(g);
           	form.debug = true;
           	
           	g.setColor(Color.ORANGE);
           	new WeightedForce(1, 0).draw(lastStatus, myWorld, g);
        	g.setColor(Color.BLACK);
        	if (safety != null && safety.getWeight() != 0)
        		safety.draw(lastStatus, myWorld, g);
        	g.setColor(Color.RED);
        	if (collision != null && collision.getWeight() != 0)
        		collision.draw(lastStatus, myWorld, g);
        	g.setColor(Color.GREEN);
        	if (alignment != null && alignment.getWeight() != 0)
        		alignment.draw(lastStatus, myWorld, g);
        	g.setColor(Color.BLUE);
        	if (centering != null && centering.getWeight() != 0)
        		centering.draw(lastStatus, myWorld, g);
        	g.setColor(Color.YELLOW);
        	if (light != null && light.getWeight() != 0)
        		light.draw(lastStatus, myWorld, g);
        	g.setColor(Color.WHITE);
        	if (total != null && total.getWeight() != 0)
        		total.draw(lastStatus, myWorld, g);
        } else
        	super.draw(g);	
    }
    
    /**
     * What are you interested in?  Here: just close lights
     *
     * @param p percept of potential target
     * @return whether it's interesting or not
     */
    @Override
    protected boolean isTarget(Percept p) {
    	return (p.getObjectCategory() == Percept.ObjectCategory.LIGHT &&
    			p.getDistance() < flocking.detectionDistance);
    }
    
    /**
     * Eat light sources
     * 
     * @param neighbor type of agent you've collided with
     * @return what you do with them
     */
    @Override
    public InteractiveBehavior behaviorOnApproach(Percept.ObjectCategory neighbor) {
    	if (neighbor == Percept.ObjectCategory.LIGHT) 
    		return InteractiveBehavior.ATTACK;
    	else
    		return InteractiveBehavior.COEXIST;
    }
    
    /**
     * Boids have three impulses:
     * - Avoiding collisions
     * - Aligning with neighbors
     * - Moving towards center of neighbors
     * These methods implement each of these calculations
     */


    protected boolean isTargetClear(Percept p) {
        if(p.getObjectCategory() == Percept.ObjectCategory.OBSTACLE ||
        p.getObjectCategory() == Percept.ObjectCategory.PREDATOR){
            /* Is agent of interest */
            if(p.getDistance() < flocking.detectionDistance){
                /* Is within detection distance */

                if(p.getAngle() >= 0){
                    if(p.getAngle() <= flocking.cone){
                    /* Is within positive detection angle */
                    return true;
                    }
                }else if(p.getAngle() >= -flocking.cone){
                    /* Is within negative detection angle */
                    return true;
                } 
            }
        }

        return false;
    }
    protected WeightedForce maintainClearance(List<Percept> ps) {
        WeightedForce mf = new WeightedForce();

        double angle            = 0;
        double distScaledWt     = flocking.obstacleWeight;
    

        for(Percept p: ps){
            if(isTargetClear(p)){
                /* Move towards cone direction inversely proportional to distance */
                if(p.getAngle() <= 0)
                    angle = flocking.cone;
                else
                    angle = -flocking.cone;
                distScaledWt = distScaledWt*(flocking.clearance/p.getDistance());
                WeightedForce clearForce = new WeightedForce(distScaledWt, angle);
                mf.addIn(clearForce);

            }
        }

        return mf;
    }

















    protected boolean isTargetBoidSep(Percept p) {
        if(p.getObjectCategory() == Percept.ObjectCategory.BOID){
            /* Is agent of interest */
            if(p.getDistance() < flocking.separationDistance){
                /* Is within detection distance */
                return true;
            }
        }

        return false;
    }

    protected WeightedForce separateFromNeighbors(List<Percept> ps) {
        WeightedForce sf = new WeightedForce();

        double angle            = 0;
        double distScaledWt     = flocking.separationWeight;

        for(Percept p: ps){
            if(isTargetBoidSep(p)){
                /* Move away from boids inversely proportional to distance */
                angle = -p.getAngle();
                distScaledWt = distScaledWt*(flocking.separationDistance/p.getDistance());
                WeightedForce sepForce = new WeightedForce(distScaledWt, angle);
                sf.addIn(sepForce);
                distScaledWt = flocking.separationWeight;
            }

        }



        return sf;
    }



















    protected boolean isTargetBoidAlign(Percept p) {
        if(p.getObjectCategory() == Percept.ObjectCategory.BOID){
            /* Is agent of interest */
            if(p.getDistance() < flocking.detectionDistance &&
                p.getDistance() > flocking.separationDistance){
                /* Is within detection distance but outside sepDistance*/
                return true;
            }
        }
        return false;
    }


    protected WeightedForce alignWithNeighbors(List<Percept> ps) {
        WeightedForce af = new WeightedForce();
        double  angle            = 0;
        double  distScaledWt     = flocking.alignmentWeight;
        int     neighborCount    = 0;

        for(Percept p: ps){
            if(isTargetBoidAlign(p)){
                neighborCount++;
                /* Move towards boid direction with alignmentWeight */
                angle = p.getOrientation();
                //distScaledWt = distScaledWt*(flocking.detectionDistance/p.getDistance());
                WeightedForce alignForce = new WeightedForce(distScaledWt, angle);
                af.addIn(alignForce);
                //distScaledWt = flocking.alignmentWeight;
            }

        }

        if(neighborCount>=1)
            af.reweight(1.0/neighborCount);

        return af;
    }

























    protected boolean isTargetBoidCenter(Percept p) {
        if(p.getObjectCategory() == Percept.ObjectCategory.BOID){
            /* Is agent of interest */
            if(p.getDistance() < flocking.detectionDistance &&
                p.getDistance() > flocking.separationDistance){
                /* Is within detection distance but outside sepDistance*/
                return true;
            }
        }

        return false;
    }

    protected WeightedForce centerOnNeighbors(List<Percept> ps) {
    	WeightedForce cf = new WeightedForce();

        double  distScaledWt     = flocking.centeringWeight;
        int     neighborCount    = 0;

        for(Percept p: ps){
            if(isTargetBoidCenter(p)){
                neighborCount++;

                cf.addIn(new WeightedForce(flocking.centeringWeight, p.getAngle()));

            }
        }

        if(neighborCount>=1){
            cf.reweight(1.0/neighborCount);
        }

        return cf;
    }

    






    /* HELPER METHODS FOR FOLLOWING LIGHT
        Attempts to find the closet light source given a set of percepts.
        Similar to methods in Follower.java
    */
    protected boolean isTargetLight(Percept p) {
        return (p.getObjectCategory() == Percept.ObjectCategory.LIGHT &&
                 p.getDistance() < flocking.detectionDistance);
    }
    protected double targetCost(Percept p) {
        return (p.getDistance());
    }
    protected Percept bestTarget(List<Percept> ps, Percept.Measure m) {
        Percept bestTarget = null;
        double bestValue = 0;

        for (Percept p: ps) {
            if (isTargetLight(p)) {
                if (bestTarget != null) {
                    double quality = m.cost(p);
                    if (quality < bestValue) {
                        bestTarget = p;
                        bestValue = quality;
                    }
                } else {
                    bestTarget = p;
                    bestValue = m.cost(p);
                }
            }
        }
        return bestTarget;
    } 
    protected Percept bestTarget(List<Percept> ps) {
        Percept.Measure m = new Percept.Measure() { 
            public double cost(Percept p) { 
                return targetCost(p); 
            }
        };
        return bestTarget(ps, m);
    }  

    /*
        Find closest light.  Create weighted force towards it.
    */
    protected WeightedForce followLight(List<Percept> ps) {
    	WeightedForce ff = new WeightedForce();

        double angle            = 0;
        double distScaledWt     = flocking.followWeight;
        Percept closestLight    = bestTarget(ps);

        if(closestLight != null)
            angle            = closestLight.getAngle();
        else
            return ff;

        distScaledWt = distScaledWt*(flocking.detectionDistance/closestLight.getDistance());
        WeightedForce lightForce = new WeightedForce(distScaledWt, angle);
        ff.addIn(lightForce);

    	return ff;
    }


    protected WeightedForce affinityForGreen(List<Percept> ps) {
        WeightedForce az = new WeightedForce();
         for(Percept p : ps)
         {
                if (p.getColor().getGreen()==254)
                {
                    az.addIn(new WeightedForce( 5*flocking.obstacleWeight*(flocking.detectionDistance/p.getDistance()),p.getAngle()*1));
                }
         }
        return az;
    }



    
    @Override
    public void deliberate(List<Percept> ps) {

	// Calculate all the forces required for this boid
        WeightedForce inertia = new WeightedForce(1,0);


        WeightedForce safetyForce;
        if (flocking.avoidsObstacles) {
        	safety = safetyForce = maintainClearance(ps);
        } else {
        	safetyForce = new WeightedForce();
        }
        
        WeightedForce collisionForce;
        if (flocking.avoidsCollisions) {
            collision = collisionForce = separateFromNeighbors(ps);
        }
        else {
            collisionForce = new WeightedForce();
        }

        WeightedForce alignmentForce;
        if (flocking.alignsWithNeighbors) {
            alignment = alignmentForce = alignWithNeighbors(ps);
        } 
        else {
            alignmentForce = new WeightedForce();
        }

        WeightedForce centeringForce;
        if (flocking.doesCentering) {
            centering = centeringForce = centerOnNeighbors(ps);
        }
        else {
            centeringForce = new WeightedForce();
        }

        WeightedForce lightForce;
        if (flocking.followsLight) {
        	light = lightForce = followLight(ps);
        }
        else {
        	lightForce = new WeightedForce();
        }

        WeightedForce affForce;
        if (true) {
            affForce = affinityForGreen(ps);
        }
        else {
            affForce = new WeightedForce();
        }
        

        WeightedForce f = new WeightedForce();

        f.addIn(inertia);  
        
        f.addIn(safetyForce);
        f.addIn(collisionForce);
        f.addIn(alignmentForce);
        f.addIn(centeringForce);
        f.addIn(lightForce);
        f.addIn(affForce);

        
        todo = new LinkedList<Intention>();
        todo.add(new Intention(Intention.ActionType.TURN, f.getAngle()));
        todo.add(new Intention(Intention.ActionType.CHANGE_SPEED, form.maxSpeedForward - status.forwardV));
    }
}





