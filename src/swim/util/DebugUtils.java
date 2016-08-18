/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.util;

import com.jme3.bounding.BoundingBox;
import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.font.Rectangle;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.BillboardControl;
import com.jme3.scene.debug.Arrow;
import com.jme3.scene.shape.Line;
import com.jme3.scene.shape.Quad;
import com.jme3.texture.Image;
import com.jme3.texture.Image.Format;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.DefaultImageRaster;
import com.jme3.texture.image.ImageRaster;
import com.jme3.util.BufferUtils;
import com.zero_separation.plugins.imagepainter.ImagePainter;
import java.awt.color.ColorSpace;
import java.nio.ByteBuffer;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import swim.core.network.ODPair;
import swim.sim.Simulator;

/**
 *
 * @author Sherif
 */
public class DebugUtils {
    
    public static void plotArrow(Vector3f absSrc, Vector3f relDst, ColorRGBA color, float width, Simulator sim) {
        
        Arrow arrow = new Arrow(relDst);
        arrow.setLineWidth(width); // make arrow thicker
        putShape(arrow, color, sim).setLocalTranslation(absSrc);
         
//        Line line = new Line(src, dst);
//        line.setLineWidth(width);
//        Geometry lineGeom = new Geometry("", line);
//        Material material = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
//        material.setColor("Color", color);
//        lineGeom.setMaterial(material);
//        sim.getRootNode().attachChild(lineGeom);
    }
    
    public static void plotArrowSequence(List<Pair> vectors, Simulator sim) {
        
        Iterator iter = vectors.iterator();
        Pair odPair;
        while(iter.hasNext()) {
            odPair = (Pair)iter.next();
            DebugUtils.plotArrow((Vector3f)odPair.key, (Vector3f)odPair.val, ColorRGBA.Blue, 3, sim);
            //System.out.println("Origin: "+ (Vector3f)odPair.t + ", Destination: "+ (Vector3f)odPair.u);
        }
        
    }
    
    public static String plotCrossHair(Vector3f location, float lineWidth, float crossHairSize, ColorRGBA crossHairColor, Simulator sim) {
        
       Line[] lines = new Line[6];
       
       lines[0] = new Line(location, location.add(Vector3f.UNIT_X.mult(crossHairSize)));
       lines[1] = new Line(location, location.subtract(Vector3f.UNIT_X.mult(crossHairSize)));
       lines[2] = new Line(location, location.add(Vector3f.UNIT_Y.mult(0.8f*crossHairSize)));
       lines[3] = new Line(location, location.subtract(Vector3f.UNIT_Y.mult(0.8f*crossHairSize)));
       lines[4] = new Line(location, location.add(Vector3f.UNIT_Z.mult(crossHairSize)));
       lines[5] = new Line(location, location.subtract(Vector3f.UNIT_Z.mult(crossHairSize)));
       
       Geometry lineGeom;
        Material material = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
        material.setColor("Color", crossHairColor);
       
        String ID = "xhair_"+System.currentTimeMillis()+"";
        
        Node crossHair = new Node(ID);
        
       for(int i = 0; i < 6;i++) {
           lines[i].setLineWidth(lineWidth);
           lineGeom = new Geometry("line_"+i, lines[i]);
           lineGeom.setMaterial(material);
           crossHair.attachChild(lineGeom);
       }
       
       sim.getRootNode().attachChild(crossHair);
       
       return ID;
    }
    
    public static void removeCrossHair(String ID, Simulator sim) {
        sim.getRootNode().detachChildNamed(ID);
    }
    
    public static void plotSpatialName(Spatial spatial, float lineWidth, ColorRGBA textColor, Simulator sim) {
        
        Texture whiteTexture = sim.getAssetManager().loadTexture("Textures/White/white_64.png");
        
        Image img = whiteTexture.getImage(); 
        int width = img.getWidth();
        int height = img.getHeight();
        
        Format imgFormat = Format.RGBA8;
        
        ByteBuffer data = BufferUtils.createByteBuffer((int)Math.ceil(imgFormat.getBitsPerPixel() / 8.0) * width * height);
        
        Image formattedImg = new Image(imgFormat, width, height, data);
        
        ImageRaster sourceReader = ImageRaster.create(img);
        ImageRaster targetWriter = ImageRaster.create(formattedImg);
        
        ColorRGBA pixelColor;
        
        for(int x = 0; x < width; x++) {
            for(int y = 0; y < height; y++) {
                pixelColor = sourceReader.getPixel(x, y);
                targetWriter.setPixel(x, y, pixelColor);
            }  
        }
        
        Texture newTexture = new Texture2D(formattedImg);
        newTexture.setMagFilter(Texture.MagFilter.Nearest);
        newTexture.setMinFilter(Texture.MinFilter.NearestNoMipMaps);
        newTexture.setAnisotropicFilter(16);
        
        Material material = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
        material.setTexture("ColorMap", newTexture);
        
        Quad quad = new Quad(2, 2);
        Geometry geom = new Geometry("Quad", quad);
        Material mat = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setTransparent(true);
        geom.setMaterial(mat);

        BitmapFont font = sim.getAssetManager().loadFont("Interface/Fonts/Default.fnt");
        BitmapText txt = new BitmapText(font);
        
        txt.setBox(new Rectangle(0,0,5,5));
        txt.setQueueBucket(RenderQueue.Bucket.Transparent);
        txt.setSize(1f);
        txt.setText(spatial.getName());
        txt.setLocalTranslation(1,1,0);
        txt.setColor(ColorRGBA.Black);
        
        Node bb = new Node("billboard");

        BillboardControl control = new BillboardControl();
        
        bb.addControl(control);
        bb.attachChild(txt);    
        
        Node n = new Node("parent");
        n.attachChild(bb);
        sim.getRootNode().attachChild(n);

        Node  n2 = new Node("parentParent");
        n2.setLocalTranslation(Vector3f.UNIT_Y.mult(5));
        n2.attachChild(n);

        ((Node)spatial).attachChild(n2);
        
    }
    
    private static Geometry putShape(Mesh shape, ColorRGBA color, Simulator sim){
        Geometry g = new Geometry("coordinate axis", shape);
        Material mat = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
        mat.getAdditionalRenderState().setWireframe(true);
        mat.setColor("Color", color);
        g.setMaterial(mat);
        sim.getRootNode().attachChild(g);
        return g;
    }
    
}
