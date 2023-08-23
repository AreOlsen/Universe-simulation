package quad;

import java.util.ArrayList;
import java.util.List;

import physics.Body;
import vecs.Vector2;

public class Quadtree {
    public double totalmass = 0d;
    public Vector2 centerOfMass = new Vector2(0, 0);
    public double timestep;
    public Vector2 upperLeft;
    public double length;
    public Quadtree parentTree;
    public List<Body> bodies = new ArrayList<>();
    public Quadtree[] subQuads = new Quadtree[4]; //0 = upperLeft, 1 = lowerLeft, 2 = upperRight, 3 = lowerRight.
    public double biggestRadius = 0;
    public int pointCount;
    public Quadtree(List<Body> bodies, int pointCount, Vector2 upperLeft, double length, double timestep){
        //Center of mass and meta data.
        this.length = length;
        this.timestep = timestep;
        this.bodies = bodies;
        this.upperLeft = upperLeft;
        this.pointCount=pointCount;
        if(!bodies.isEmpty()){
            for(Body b : bodies){
                this.totalmass+=b.mass;
                this.centerOfMass=Vector2.Add(this.centerOfMass, Vector2.Product(b.mass, b.position));
                this.biggestRadius=Math.max(this.biggestRadius,b.radius);
            }
            if(this.totalmass!=0){
                this.centerOfMass=Vector2.Product(1/this.totalmass, this.centerOfMass);
            }
        }

        //Subquads.
        if(bodies.size()>pointCount){
            //Define bodies.
            List<Body> upperRightBodies = new ArrayList<Body>();
            List<Body> lowerRightBodies = new ArrayList<Body>();
            List<Body> upperLeftBodies = new ArrayList<Body>();
            List<Body> lowerLeftBodies = new ArrayList<Body>();

            //Categorize bodies.
            Vector2 upperRightCorner = Vector2.Add(upperLeft, new Vector2(length/2d, 0d));
            Vector2 lowerRightCorner = Vector2.Add(upperLeft, new Vector2(length/2d, -length/2d));
            Vector2 upperLeftCorner = upperLeft;
            Vector2 lowerLeftCorner = Vector2.Add(upperLeft, new Vector2(0, -length/2d));
            for(Body b : bodies){
                if(Quadtree.Inside(upperLeftCorner, length/2d, b.position)){
                    upperLeftBodies.add(b);
                } else if(Quadtree.Inside(lowerLeftCorner, length/2d, b.position)){
                    lowerLeftBodies.add(b);
                } else if(Quadtree.Inside(upperRightCorner, length/2d, b.position)){
                    upperRightBodies.add(b);
                } else {
                    lowerRightBodies.add(b);
                }
            }

            //Create quads.
            subQuads[0] = new Quadtree(upperLeftBodies, pointCount, upperLeftCorner, length/2,timestep);
            subQuads[1] = new Quadtree(lowerLeftBodies, pointCount, lowerLeftCorner, length/2,timestep);
            subQuads[2] = new Quadtree(upperRightBodies, pointCount, upperRightCorner, length/2,timestep);
            subQuads[3] = new Quadtree(lowerRightBodies, pointCount, lowerRightCorner, length/2,timestep);
            for(int i = 0; i < 4;i++){
                subQuads[i].parentTree=this;
            }
        }
    }

    public static boolean Inside(Vector2 upperLeft, double len, Vector2 point){
        boolean xInside = ((point.x>=upperLeft.x) && (point.x<=(upperLeft.x+len)));
        boolean yInside = ((point.y<=upperLeft.y) && (point.y>=(upperLeft.y-len)));
        if (xInside && yInside) {
            return true;
        }
        return false;
    }

    public static double FindMaxLength(Vector2 upperLeft, List<Body> bodies){
        double len = 0;
        for(Body b : bodies){
            Vector2 pos = Vector2.Add(b.position, new Vector2(b.radius, -b.radius));
            Vector2 diff = Vector2.Subtract(pos, upperLeft);
            len = Math.max(Math.max(Math.abs(diff.x),Math.abs(diff.y)),len);
        }
        return len;
    }

    public static Vector2 FindMaxUpperLeft(List<Body> bodies){
        Vector2 upLeft = new Vector2(Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
        for(Body b : bodies){
            upLeft.y=Math.max(b.position.y+b.radius,upLeft.y);
            upLeft.x=Math.min(b.position.x-b.radius,upLeft.x);
        }
        return upLeft;
    }

    public List<Body> FindAttractionBodies(Body body, double cutoff){
        List<Body> attracBodies = new ArrayList<>();
        for(int i = 0; i < 4; i++){
            //If no subquads or they are empty.
            if(subQuads[i]==null){
                continue;
            } else if(subQuads[i].bodies.size()==0){
                continue;
            }
            
            //If they exceed cutoff we just get the avg.
            Vector2 diff = Vector2.Subtract(body.position, subQuads[i].centerOfMass);            
            if(diff.AbsoluteValue()>=cutoff){
                Vector2 emptyVec = new Vector2(0, 0);
                Body temp = new Body(subQuads[i].totalmass, 0, subQuads[i].centerOfMass, emptyVec, emptyVec, subQuads[i].timestep);
                attracBodies.add(temp);

            } else {
                //Else we add all of them.
                attracBodies.addAll(subQuads[i].bodies);
            }
        }
        return attracBodies;
    }



    public List<Body> Query(Vector2 position, double radius, boolean ifTotalInside){
        if(ifTotalInside){
            return this.bodies;
        }
        
        List<Body> bodies = new ArrayList<>();

        boolean insideX = (((position.x-radius)>=this.upperLeft.x) || ((position.x+radius))<=(this.upperLeft.x+length));
        boolean insideY = (((position.y+radius)<=this.upperLeft.y) || ((position.y-radius))>=(this.upperLeft.y-length));
        
        //If not inside, we return empty array.
        if(!insideX && !insideY){
            return bodies;
        }

        //If no subs we get the ones inside.
        if(subQuads[0]==null){
            this.bodies.parallelStream().forEach(b -> {
                double diff = Vector2.Subtract(b.position, position).AbsoluteValue();
                if(diff<=radius){
                    bodies.add(b);
                }
            });
            return bodies;
        }

        //If there are subs we concat the bods from those.
        for(int i = 0; i < 4; i++){
            List<Body> bods = subQuads[i].Query(position, radius, insideX && insideY);
            bodies.addAll(bods);
        }
        return bodies;
    }
}
