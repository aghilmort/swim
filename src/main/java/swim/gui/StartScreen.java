/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.gui;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.input.FlyByCamera;
import com.jme3.input.InputManager;
import com.jme3.renderer.Camera;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;
import de.lessvoid.nifty.Nifty;
import de.lessvoid.nifty.NiftyEventSubscriber;
import de.lessvoid.nifty.controls.CheckBox;
import de.lessvoid.nifty.controls.CheckBoxStateChangedEvent;
import de.lessvoid.nifty.controls.DropDown;
import de.lessvoid.nifty.controls.DropDownSelectionChangedEvent;
import de.lessvoid.nifty.controls.Label;
import de.lessvoid.nifty.controls.Slider;
import de.lessvoid.nifty.controls.SliderChangedEvent;
import de.lessvoid.nifty.controls.TextField;
import de.lessvoid.nifty.controls.TextFieldChangedEvent;
import de.lessvoid.nifty.elements.Element;
import de.lessvoid.nifty.screen.Screen;
import de.lessvoid.nifty.screen.ScreenController;
import swim.algorithm.integration.IntegrationAlgorithms;
import swim.algorithm.search.SearchAlgorithms;
import swim.algorithm.taskalloc.TaskAllocAlgorithms;
import swim.sim.Simulator;
import swim.sim.uwswarm.UWVehicleControl;

/**
 *
 * @author Sherif
 */
public class StartScreen extends AbstractAppState implements ScreenController {
    
    // Essential App variables
    private Simulator         sim;
    private Node              rootNode;
    private Node              guiNode;
    private AssetManager      assetManager;
    private AppStateManager   stateManager;
    private InputManager      inputManager;
    private ViewPort          viewPort;
    private BulletAppState    physics;
    private FlyByCamera       flyCam;
    private Camera            cam;
    private BulletAppState    bulletAppState;
    
    private Node localRootNode = new Node("Start Screen RootNode");
    private Node localGuiNode = new Node("Start Screen GuiNode");
    
    private Nifty nifty;
    private Screen screen;
    
    public StartScreen(Simulator sim, Nifty nifty) {
        this.sim = (Simulator) sim;

        this.rootNode     = this.sim.getRootNode();
        this.guiNode      = this.sim.getGuiNode();
        this.assetManager = this.sim.getAssetManager();
        this.stateManager = this.sim.getStateManager();
        this.inputManager = this.sim.getInputManager();
        this.viewPort     = this.sim.getViewPort();
        this.physics      = this.stateManager.getState(BulletAppState.class);
        this.cam          = this.sim.getCamera();
        this.flyCam       = this.sim.getFlyByCamera();
        this.bulletAppState = ((Simulator)this.sim).getBulletAppState();
        
        this.nifty = nifty;
    }
    
    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        super.initialize(stateManager, app);
        
        //TODO: initialize your AppState, e.g. attach spatials to rootNode
        //this is called on the OpenGL thread after the AppState has been attached
        
        rootNode.attachChild(localRootNode);
        guiNode.attachChild(localGuiNode);
    }
    
    @Override
    public void update(float tpf) {
        //TODO: implement behavior during runtime
    }
    
    @Override
    public void cleanup() {
        rootNode.detachChild(localRootNode);
        guiNode.detachChild(localGuiNode);

        super.cleanup();
        //TODO: clean up what you initialized in the initialize method,
        //e.g. remove all spatials from rootNode
        //this is called on the OpenGL thread after the AppState has been detached
    }

    @Override
    public void bind(Nifty nifty, Screen screen) {
        
        this.screen = screen;
        
        DropDown searchDropDown = findDropDownControl("dropdown_Search_Sim");
        if (searchDropDown != null) {
            
            SearchAlgorithms[] searchAlgs = sim.getAvailableSearchAlgList();
            
            for(SearchAlgorithms alg : searchAlgs) {
                searchDropDown.addItem(alg.longName());
            }

            searchDropDown.selectItemByIndex(UWVehicleControl.getActiveSearchAlgorithm().index());
        }
        
        DropDown taskAllocDropDown = findDropDownControl("dropdown_Taskalloc_Sim");
        if (taskAllocDropDown != null) {
            
            TaskAllocAlgorithms[] taskAllocAlgs = sim.getAvailableTaskAllocAlgList();
            
            for(TaskAllocAlgorithms alg : taskAllocAlgs) {
                taskAllocDropDown.addItem(alg.longName());
            }

            taskAllocDropDown.selectItemByIndex(UWVehicleControl.getActiveTaskAllocAlgorithm().index());
        }
        
        CheckBox checkBox = findCheckBoxControl("chkbx_veh_path_debug");
        if (checkBox != null) {
            checkBox.setChecked(UWVehicleControl.getVehiclePathDebug());
        }
        
        DropDown integrationDropDown = findDropDownControl("dropdown_Integ_Sim");
        if (integrationDropDown != null) {
            
            IntegrationAlgorithms[] integAlgs = sim.getAvailableIntegAlgList();
            
            for(IntegrationAlgorithms alg : integAlgs) {
                integrationDropDown.addItem(alg.longName());
            }

            integrationDropDown.selectItemByIndex(UWVehicleControl.getActiveIntegAlgorithm().index());
        }
        
    }

    @Override
    public void onStartScreen() {
    }

    @Override
    public void onEndScreen() {
    }
    
    /** custom methods */ 
    public void modeButtonHandler(String simID) {
        
        int simulationID = Integer.valueOf(simID);
        
        sim.initSimulator(simulationID);
        this.cleanup();
    }
    
    @NiftyEventSubscriber(pattern="dropdown_.*")
    public void onDropDownSelectionChangedEvent(String id, DropDownSelectionChangedEvent event) {
        //System.out.println("Selected: "+event.getSelection()+", "+event.getSelectionItemIndex());
        if( id.equals("dropdown_Search_Sim") ) {
            UWVehicleControl.setSearchAlgorithm(event.getSelectionItemIndex());
        } else if( id.equals("dropdown_Taskalloc_Sim") ) {
            UWVehicleControl.setTaskAllocAlgorithm(event.getSelectionItemIndex());
        } else if( id.equals("dropdown_Integ_Sim") ) {
            UWVehicleControl.setIntegAlgorithm(event.getSelectionItemIndex());
            System.out.println(event.getSelection().toString());
        }
    }
    
    @NiftyEventSubscriber(pattern="slider_.*")
    public void onSliderChangedEvent(String id, SliderChangedEvent event) {
        //System.out.println("Input: "+event.getValue());
        Simulator.setNumberOfAgents((int)event.getValue());
        
        Label numAgentsLabel = findLabelControl("label_Num_Agents");     
        if(numAgentsLabel != null) {
            numAgentsLabel.setText("Number of AUVs: "+(int)event.getValue()+"\n");
        }
    }
    
//    public void debugCheckHandler(String flagID, String checkID) {
//        int fID = Integer.valueOf(flagID);
//        
//        ((Simulator)app).toggleFlag(fID);
//
//        Screen thisScreen = nifty.getScreen("start_screen");
//        
//        if(thisScreen != null) {
//            CheckBox checkBox = (CheckBox)thisScreen.findElementById(checkID);
//            if(checkBox != null) {
//                checkBox.toggle();
//            }
//        }
//       
//        this.cleanup();
//    }
    
    @NiftyEventSubscriber(pattern="chkbx_.*")
    public void onCheckBoxStateChanged(String id, CheckBoxStateChangedEvent event) {
        CheckBox checkBox = event.getCheckBox();
        if(checkBox.isChecked()) {
            checkBox.uncheck();
        } else {
            checkBox.check();
        }
    }
    
    private DropDown findDropDownControl(final String id) {
        return screen.findNiftyControl(id, DropDown.class);
    }
    
    private CheckBox findCheckBoxControl(final String id) {
        return screen.findNiftyControl(id, CheckBox.class);
    }
    
    private Label findLabelControl(final String id) {
        return screen.findNiftyControl(id, Label.class);
    }
    
    private Slider findTextFieldControl(final String id) {
        return screen.findNiftyControl(id, Slider.class);
    }
    
    
}
