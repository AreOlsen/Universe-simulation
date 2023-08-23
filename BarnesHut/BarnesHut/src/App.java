import renderer.Camera;
import quad.Quadtree;
import vecs.Vector2;
import physics.Body;

import java.util.ArrayList;
import java.util.List;

public class App {
    public static void main(String[] args) throws Exception {
        List<Body> bodies = new ArrayList<>();
        double timestep = 0.1;
        for(int i = 0; i < 100; i++){
            double theta = Math.PI*2*Math.random();
            
            double y = Math.sin(theta);
            double x = Math.cos(theta);

            Vector2 radius = new Vector2(x, y);
            Vector2 location = Vector2.Product(Math.random()*Math.pow(10,6)+Math.pow(10,5), radius);
            Vector2 emptyVector = new Vector2(0, 0); 

            Body newBody = new Body(5.972*Math.pow(10,24), Math.pow(10,4), location, emptyVector, emptyVector, timestep);
            bodies.add(newBody);
        }
        Vector2 upperLeft = Quadtree.FindMaxUpperLeft(bodies);
        double quadLength = Quadtree.FindMaxLength(upperLeft, bodies);
        Quadtree quad = new Quadtree(bodies, 5, upperLeft, quadLength, timestep,0,(int)(Math.log(bodies.size())/Math.log(6))+1);

        int WIDTH = 1440;
        int HEIGHT = 810;

        Camera renderingEngine = new Camera(quad, 1d, WIDTH, HEIGHT, timestep, false, 0.9999999);
    }
}
