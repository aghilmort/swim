/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.input.FlyByCamera;
import com.jme3.input.InputManager;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import java.util.ArrayList;
import java.util.List;
import swim.util.DebugUtils;

/**
 *
 * @author Sherif
 */
public class GeneralTestsMode extends AbstractAppState {
    
    // Essential App variables
    private SimpleApplication app;
    private Node              rootNode;
    private AssetManager      assetManager;
    private AppStateManager   stateManager;
    private InputManager      inputManager;
    private ViewPort          viewPort;
    private BulletAppState    physics;
    private FlyByCamera       flyCam;
    private Camera            cam;
    private BulletAppState    bulletAppState;
    
    
    // =========================================================================
    // Config
    // =========================================================================
    private boolean VIEW_GRID = false;
    // =========================================================================
    // Config
    // =========================================================================
    
    // Control variables
    private boolean componentsAdded = false;
    
    @Override
    public void initialize(AppStateManager stateManager, Application app) {

        super.initialize(stateManager, app);
        this.app = (SimpleApplication) app;

        this.rootNode     = this.app.getRootNode();
        this.assetManager = this.app.getAssetManager();
        this.stateManager = this.app.getStateManager();
        this.inputManager = this.app.getInputManager();
        this.viewPort     = this.app.getViewPort();
        this.physics      = this.stateManager.getState(BulletAppState.class);
        this.cam          = this.app.getCamera();
        this.flyCam       = this.app.getFlyByCamera();
        this.bulletAppState = ((Simulator)this.app).getBulletAppState();
    }
    
    @Override
    public void update(float tpf) {
        if(!componentsAdded && this.isEnabled()) {
            
            // Add components here ...
            if(VIEW_GRID) {
                drawGrid();
            }
            componentsAdded = true;
        }
    }    
    
    public void drawGrid() {
        for(float x = -128; x <= 128; x+=4) {
           for(float z = -128; z <= 128; z+=4) {
                DebugUtils.plotCrossHair(new Vector3f(x, 0.0f, z), 2, 0.5f, ColorRGBA.Red, (Simulator)app);
           }     
        }
    }
    
}
