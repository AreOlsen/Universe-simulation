package renderer;

import javax.swing.JFrame;
import javax.swing.JPanel;
import java.util.List;
import java.awt.*;
import java.awt.event.*;
import quad.Quadtree;
import vecs.Vector2;
import physics.Body;

public class Camera{
    double zoomfactor = 1;
    Vector2 focusPoint = new Vector2(0, 0);
    double cameraSpeed;
    Quadtree quad;
    boolean focusCenter = false;
    double zoomFacDiff = 0;
    double currentFPS = 0;

    JFrame frame = new JFrame("Are Olsen's Universe Simulation");
    Container pane;
    JPanel panel;
    public Camera(Quadtree quad, double cameraSpeed, int width, int height, double timestep, boolean focusCenter, double cofRef){
        this.cameraSpeed = cameraSpeed;
        this.quad = quad;
        this.focusCenter = focusCenter;
        //Establish panel.
        pane = frame.getContentPane();
        pane.setLayout(new BorderLayout());
        panel = new JPanel(){
            public void paintComponent(Graphics g) {
                //Establish black background
                Graphics2D g2 = (Graphics2D) g;
                g2.setColor(Color.BLACK);
                g2.fillRect(0, 0, getWidth(), getHeight());
                
                //Render all bodies.
                for(Body b : quad.bodies){
                    g2.setColor(b.col);
                    //If body is inside the camera size, takes into account zoom.
                    Vector2 leftBottomCameraCorner = Vector2.Add(focusPoint, new Vector2(-(getWidth())/2, -(getHeight())/2));
                    Vector2 cords = Vector2.Subtract(leftBottomCameraCorner, Vector2.Add(b.position,new Vector2(-b.radius,b.radius)));
                    cords = Vector2.Product(1/zoomfactor, cords);
                    int rad = (int)(Math.max(1,b.radius*2/zoomfactor));
                    g2.fillOval((int)cords.x, panel.getHeight()-(int)cords.y, rad, rad);
                }

                // Draw FPS in the left bottom corner
                int padding = 15;
                String fpsText = String.format("FPS: %.2f", currentFPS);
                g2.setColor(Color.WHITE);
                FontMetrics metrics = g2.getFontMetrics();
                int textHeight = metrics.getHeight();
                int x = padding;
                int y = getHeight() - padding - textHeight;
                g2.drawString(fpsText, x, y);
            }
        };
        pane.add(panel, BorderLayout.CENTER);
        frame.setSize(width, height);
        frame.setVisible(true);
        panel.setFocusable(true);
        panel.requestFocusInWindow();

        panel.addKeyListener(new KeyAdapter() {
            @Override
            public void keyReleased(KeyEvent ke) { 
                if(ke.getKeyCode()==KeyEvent.VK_1){
                    Zoom(true,0.5, focusPoint, panel);
                } else if(ke.getKeyCode()==KeyEvent.VK_2){
                    Zoom(false,0.5, focusPoint, panel);
                } else if(ke.getKeyCode()==KeyEvent.VK_3){
                    FlipFocusCenterBool();
                }
            } 
        });

        class MouseDragState {
            public MouseEvent lastPress;
        }
        
        MouseDragState dragState = new MouseDragState();

        panel.addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent me){
                dragState.lastPress=me;
                System.out.println("Started drag.");
            }
        });

        panel.addMouseMotionListener(new MouseMotionAdapter(){
            @Override
            public void mouseDragged(MouseEvent me){
                if(dragState.lastPress!=null){
                    System.out.println("Ended drag");
                    double dX = -me.getX()+dragState.lastPress.getX();
                    double dY = me.getY()-dragState.lastPress.getY();
                    System.out.println(dX);
                    System.out.println(dY);
                    Vector2 movement = Vector2.Product(cameraSpeed*zoomfactor, new Vector2(dX, dY));
                    if(!focusCenter){
                        focusPoint = Vector2.Add(focusPoint,movement);
                    }
                    dragState.lastPress=me;
                    panel.repaint();
                }
            }
        });

        double prevTime = (double)System.nanoTime();
        for(;;){
            double nowTime = (double)System.nanoTime();
            double timeDelta = (nowTime-prevTime)*Math.pow(10,-9);
            currentFPS = 1/timeDelta;
            prevTime=nowTime;
            System.out.println(timeDelta);
            Update(timestep*timeDelta, quad.length/2, cofRef); 
        }

    }

    public void Zoom(boolean zoomIn, double fac, Vector2 focusPoint, JPanel panel){
        //Store the old zoom factor for later use
        double oldZoomFactor = zoomfactor;

        // Update the zoom factor
        zoomfactor *= (zoomIn) ? fac : 1/fac;

        // Calculate the difference in zoomed width and height
        double widthDiff = panel.getWidth() * (zoomfactor - oldZoomFactor);
        double heightDiff = panel.getHeight() * (zoomfactor - oldZoomFactor);
        this.zoomFacDiff += (zoomfactor - oldZoomFactor);
        // Calculate the new focus point to keep it centered
        Vector2 newFocusPoint = new Vector2(
            focusPoint.x - widthDiff / 2,
            focusPoint.y - heightDiff / 2
        );

        // Update the focus point
        this.focusPoint = newFocusPoint;
        panel.repaint();
    }

    public void FlipFocusCenterBool(){
        this.focusCenter=!this.focusCenter;
    }


    public void SetFocusCenter(boolean set){
        if(set){
            Vector2 newFocusPoint = new Vector2(
                quad.centerOfMass.x - this.zoomFacDiff*pane.getWidth()/2,
                quad.centerOfMass.y - this.zoomFacDiff*pane.getHeight()/2
            );
            this.focusPoint = newFocusPoint;
        }
    }

    public void Update(double timestep, double cutoff, double cofRef){
        UpdateAccVelPos(timestep, cutoff);
        SolveCollisions(8, quad.bodies, cofRef);
        UpdateQuad(timestep);
        SetFocusCenter(focusCenter);
        panel.repaint();
    }
    
    public void UpdateQuad(double timestep){
        Vector2 upLeft = Quadtree.FindMaxUpperLeft(quad.bodies);
        double newLength = Quadtree.FindMaxLength(upLeft, quad.bodies);
        this.quad = new Quadtree(quad.bodies, quad.pointCount, upLeft, newLength, timestep);
    }


    public void UpdateAccVelPos(double timestep, double cutoff){
        quad.bodies.parallelStream().forEach(b -> {
            //Traverse down quad tree.
            List<Body> attractionBodies = quad.FindAttractionBodies(b, cutoff);
            b.UpdateAcceleration(attractionBodies);
            b.UpdateVelocity(timestep);
            b.UpdatePosition(timestep);
            b.UpdateColour();
        });
    }

    public void SolveCollisions(int subStepCount, List<Body> bodies, double cof){
        for(int i = 0; i < subStepCount; i++){ 
            bodies.parallelStream().forEach(b -> {
                List<Body> cellBodies = quad.Query(b.position, b.radius+quad.biggestRadius, false);
                for(Body b2 : cellBodies){
                    if(b2==b){
                        continue;
                    }

                    double distance = Vector2.Subtract(b.position, b2.position).AbsoluteValue();
                    if(distance<(b.radius+b2.radius)){
                        b.Collision(b2, cof);
                    }
                } 
            });
        }
    }

}
