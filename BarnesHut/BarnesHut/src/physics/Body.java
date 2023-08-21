package physics;

import java.util.List;
import vecs.Vector2;
import java.awt.Color;

public class Body {
    public double mass; //kg
    public double radius; //m
    public Vector2 position; //m,m,m
    public Vector2 velocity; //m/s, m/s, m/s
    public Vector2 acceleration; //m/s^2, m/s^2, m/s^2.
    public Color col = new Color(0, 255, 0);
    public double heat = 1;

    public Body(double mass, double radius, Vector2 position, Vector2 velocity, Vector2 acceleration, double timestep){
        this.position=position;
        this.mass=mass;
        this.radius=radius;
        this.velocity=velocity;
        this.acceleration=acceleration;
    }

    private Vector2 GravityAcc(List<Body> bodies){
        Vector2 acc = new Vector2(0,0);
        for(int i = 0; i < bodies.size(); i++){
            if(bodies.get(i)==this){
                continue;
            }
            Vector2 distance = Vector2.Subtract(position,bodies.get(i).position);
            double gravityConstant = 6.67430*Math.pow(10,-11);
            double absGrav = gravityConstant*bodies.get(i).mass/(Math.pow(distance.AbsoluteValue(),2));
            Vector2 lerp = Vector2.UnitVector(distance);
            acc.x+=absGrav*lerp.x;
            acc.y+=absGrav*lerp.y;
        }
        return acc;
    }

    public void UpdateColour(){
        heat*=0.5;
        col = new Color((int)Math.min(255,255*heat/15), (int)Math.min(255,Math.max(255*(1-heat/15),0)), 0);
    }

    public double Momentum(double vel, double mass){
        return mass*vel;
    }

    public void Collision(Body body, double cof){
        //If colliding.
        Vector2 normal = Vector2.Subtract(position, body.position);
        double distance = normal.AbsoluteValue();

        //Push back both bodies.
        double overlap = (radius+body.radius)-distance;
        Vector2 pushBackUnitVector = Vector2.UnitVector(normal);
        position = Vector2.Subtract(Vector2.Product(overlap/2, pushBackUnitVector), position);
        body.position = Vector2.Add(Vector2.Product(overlap/2, pushBackUnitVector), body.position);
        

        //Update velocities. https://www.vobarian.com/collisions/2dcollisions2.pdf
        Vector2 unitNormal = Vector2.UnitVector(normal);
        Vector2 unitTangent = Vector2.UnitVector(Vector2.Normal(normal));

        Vector2 v1 = velocity;
        Vector2 v2 = body.velocity;
        double v1n = Vector2.DotProduct(unitNormal, v1);
        double v1t = Vector2.DotProduct(unitTangent, v1);

        double v2n = Vector2.DotProduct(unitNormal, v2);
        double v2t = Vector2.DotProduct(unitTangent, v2);

        double v1nd = (v1n*(mass-body.mass)+2*body.mass*v2n)/(mass+body.mass);
        double v2nd = (v2n*(body.mass-mass)+2*mass*v1n)/(mass+body.mass);

        Vector2 v1ndv = Vector2.Product(v1nd, unitNormal);
        Vector2 v2ndv = Vector2.Product(v2nd, unitNormal);

        Vector2 v1tdv = Vector2.Product(v1t, unitTangent);
        Vector2 v2tdv = Vector2.Product(v2t, unitTangent);

        //Heat update.
        body.heat+=0.0001*(Vector2.Subtract(Vector2.Product(cof, Vector2.Add(v1ndv,v1tdv)),velocity).AbsoluteValue());
        heat+=0.0001*(Vector2.Subtract(Vector2.Product(cof, Vector2.Add(v2ndv,v2tdv)),body.velocity).AbsoluteValue());

        //Speed update.
        velocity=Vector2.Product(cof, Vector2.Add(v1ndv,v1tdv));
        body.velocity=Vector2.Product(cof, Vector2.Add(v2ndv,v2tdv));
    }

    //Euler integration.
    public void UpdatePosition(double timestep){
        position = Vector2.Add(position,Vector2.Add(Vector2.Product(timestep, velocity), Vector2.Product(0.5*timestep*timestep, acceleration)));
    }

    public void UpdateVelocity(double timestep){
        velocity=Vector2.Add(velocity,Vector2.Product(timestep, acceleration));
    }

    public void UpdateAcceleration(List<Body> bodies){
        acceleration = GravityAcc(bodies);
    }
}
