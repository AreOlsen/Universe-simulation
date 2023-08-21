package vecs;

public class Vector2{
    public double x,y;
    public Vector2(double x, double y){
        this.x=x;
        this.y=y;
    }

    public double AbsoluteValue(){
        return Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
    }

    public static Vector2 Add(Vector2 vec, Vector2 vec1){
        return new Vector2(vec1.x+vec.x, vec1.y+vec.y);
    }

    public static Vector2 Subtract(Vector2 vec, Vector2 vec1){
        return new Vector2(vec1.x-vec.x, vec1.y-vec.y);
    }

    public static double DotProduct(Vector2 vec, Vector2 vec1){
        double val = vec1.x*vec.x+vec1.y*vec.y;
        return val;
    }

    public static Vector2 Product(double constant, Vector2 vec){
        return new Vector2(vec.x*constant, vec.y*constant);
    }

    public static double Angle(Vector2 vec, Vector2 vec1){
        return Math.asin((DotProduct(vec, vec1))/(vec.AbsoluteValue()*vec1.AbsoluteValue()));
    }

    public static Vector2 UnitVector(Vector2 vec){
        double tot = vec.AbsoluteValue();
        return new Vector2(vec.x/tot, vec.y/tot);
    }
    
    public static Vector2 Normal(Vector2 vec){
        return new Vector2(-vec.y,vec.x);
    }
}