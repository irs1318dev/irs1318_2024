package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.g2d.Batch;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.scenes.scene2d.Actor;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.CheckBox;
import com.badlogic.gdx.scenes.scene2d.ui.Label;
import com.badlogic.gdx.scenes.scene2d.ui.ScrollPane;
import com.badlogic.gdx.scenes.scene2d.ui.SelectBox;
import com.badlogic.gdx.scenes.scene2d.ui.Skin;
import com.badlogic.gdx.scenes.scene2d.ui.Slider;
import com.badlogic.gdx.scenes.scene2d.ui.SplitPane;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.scenes.scene2d.utils.ChangeListener;
import com.badlogic.gdx.utils.Disposable;
import com.badlogic.gdx.utils.viewport.ExtendViewport;

import frc.lib.CoreRobot;
import frc.lib.driver.AnalogAxis;
import frc.lib.driver.IButtonMap;
import frc.lib.driver.UserInputDeviceButton;
import frc.lib.driver.descriptions.AnalogOperationDescription;
import frc.lib.driver.descriptions.DigitalOperationDescription;
import frc.lib.driver.descriptions.MacroOperationDescription;
import frc.lib.driver.descriptions.OperationDescription;
import frc.lib.helpers.Pair;
import frc.lib.robotprovider.Point2d;
import frc.robot.mechanisms.ArmKinematicsCalculator;
import frc.robot.mechanisms.ArmKinematicsCalculator.ExtensionType;
import frc.robot.simulation.RobotSimulator;
import frc.robot.simulation.SimulatorBase;
import space.earlygrey.shapedrawer.ShapeDrawer;

public class FauxbotJointSpaceReachScreen implements Screen
{
    private final FauxbotGame game;

    private final Stage stage;
    private final Skin skin;
    private final JointSpaceDiagram jointSpaceDiagram;

    public FauxbotJointSpaceReachScreen(final FauxbotGame game)
    {
        this.game = game;

        this.stage = new Stage(new ExtendViewport(900, 750));
        Gdx.input.setInputProcessor(this.stage);

        this.skin = new Skin(Gdx.files.internal("skin/irs1318skin.json"));

        Table primaryTable = new Table(this.skin);
        primaryTable.setFillParent(true);
        ////primaryTable.setDebug(true);

        Label title = new Label("JointSpace Reach diagram", this.skin, "title");
        primaryTable.add(title).pad(20).top();
        primaryTable.row();

        Table scrollTable = new Table(this.skin);
        ScrollPane scrollPane = new ScrollPane(scrollTable, this.skin);
        ////scrollTable.setDebug(true);

        this.jointSpaceDiagram = new JointSpaceDiagram();
        scrollTable.add(this.jointSpaceDiagram).center().expand();

        primaryTable.add(scrollPane).expand().pad(5).fill();
        this.stage.addActor(primaryTable);
    }

    @Override
    public void render(float delta)
    {
        Gdx.gl.glClearColor(Color.PURPLE.r, Color.PURPLE.g, Color.PURPLE.b, Color.PURPLE.a);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        this.stage.act(delta);
        this.stage.draw();
    }

    @Override
    public void resize(int width, int height)
    {
        // update the viewport
        this.stage.getViewport().update(width, height, true);
    }

    @Override
    public void dispose()
    {
        this.stage.dispose();
        this.jointSpaceDiagram.dispose();
    }

    @Override
    public void pause()
    {
    }

    @Override
    public void resume()
    {
    }

    @Override
    public void show()
    {
    }

    @Override
    public void hide()
    {
    }

    public class JointSpaceDiagram extends Actor implements Disposable
    {
        private static final Color LIGHT_RED = Color.RED.cpy().lerp(Color.WHITE, 0.5f);
        private static final Color LIGHT_SALMON = Color.SALMON.cpy().lerp(Color.WHITE, 0.5f);
        private static final Color LIGHT_MAGENTA = Color.MAGENTA.cpy().lerp(Color.WHITE, 0.5f);
        private static final HashMap<ExtensionType, Color> extensionTypeColors = new HashMap<ExtensionType, Color>()
        {
            {
                put(ExtensionType.None, Color.GREEN);
                put(ExtensionType.Back, Color.CORAL);
                put(ExtensionType.Robot, Color.RED);
                put(ExtensionType.Ground, LIGHT_RED);
                put(ExtensionType.TopCrazy, Color.BROWN);
                put(ExtensionType.TopBoth, Color.SALMON);
                put(ExtensionType.TopIntakeSide, LIGHT_SALMON);
                put(ExtensionType.TopShooterSide, LIGHT_SALMON);
                put(ExtensionType.TopNone, Color.SALMON);
                put(ExtensionType.FrontBoth, Color.MAGENTA);
                put(ExtensionType.FrontIntakeTop, LIGHT_MAGENTA);
                put(ExtensionType.FrontIntakeBottom, LIGHT_MAGENTA);
                put(ExtensionType.FrontNone, Color.MAGENTA);
            }
        };

        private Pixmap pixMap;
        private Texture currentPixMapTexture;
        private List<Point2d> targetPoints;
        private Texture target;

        public JointSpaceDiagram()
        {
            double mapWidth = 2.0 * (Math.round(TuningConstants.ARM_SHOULDER_MAX_POSITION) - Math.round(TuningConstants.ARM_SHOULDER_MIN_POSITION));
            double mapHeight = Math.round(TuningConstants.ARM_WRIST_MAX_POSITION) - Math.round(TuningConstants.ARM_WRIST_MIN_POSITION);

            this.pixMap = new Pixmap((int)mapWidth, (int)mapHeight, Format.RGB888);
            this.pixMap.setColor(Color.WHITE);
            this.pixMap.fill();
            this.currentPixMapTexture = new Texture(this.pixMap);

            this.target = new Texture(Gdx.files.internal("images/target.png"));

            Color currentColor = Color.GREEN;
            this.pixMap.setColor(currentColor);

            double xOffset = -2.0 * Math.round(TuningConstants.ARM_SHOULDER_MIN_POSITION);
            double yOffset = -1.0 * Math.round(TuningConstants.ARM_WRIST_MIN_POSITION);
            Pair<Double, Double> result = new Pair<Double, Double>(0.0, 0.0);
            ArmKinematicsCalculator calculator = new ArmKinematicsCalculator(TuningConstants.ARM_SHOULDER_MIN_POSITION, TuningConstants.ARM_WRIST_MIN_POSITION);
            for (double shoulderPosition = Math.round(TuningConstants.ARM_SHOULDER_MIN_POSITION); shoulderPosition < Math.round(TuningConstants.ARM_SHOULDER_MAX_POSITION); shoulderPosition += 0.5)
            {
                for (double wristPosition = Math.round(TuningConstants.ARM_WRIST_MIN_POSITION); wristPosition < Math.round(TuningConstants.ARM_WRIST_MAX_POSITION); wristPosition += 1.0)
                {
                    calculator.calculateArmLimits(shoulderPosition, wristPosition, result);
                    Color newColor = JointSpaceDiagram.extensionTypeColors.get(calculator.getExtensionType());
                    if (newColor != currentColor)
                    {
                        this.pixMap.setColor(newColor);
                        currentColor = newColor;
                    }

                    // System.out.println("Drawing " + currentColor.toString() + " pixel at (" + (int)(xOffset + shoulderPosition * 2.0) + ", " + (int)(mapHeight - (yOffset + wristPosition * 2.0)) + ")");
                    this.pixMap.drawPixel((int)(xOffset + shoulderPosition * 2.0), (int)(mapHeight - (yOffset + wristPosition)));
                }
            }

            this.currentPixMapTexture.draw(this.pixMap, 0, 0);

            Set<ArmKinematicsCalculator.ArmGraphNode> allNodes = ArmKinematicsCalculator.getAllGraphNodes();
            int targetWidth = (int)(this.target.getWidth() / 2.0);
            int targetHeight = (int)(this.target.getHeight() / 2.0);
            this.targetPoints = new ArrayList<Point2d>(allNodes.size());
            for (ArmKinematicsCalculator.ArmGraphNode node : allNodes)
            {
                System.out.println(node.shoulderAngle + " " + node.wristAngle);
                this.targetPoints.add(
                    new Point2d(
                        xOffset + 2.0 * node.shoulderAngle - targetWidth,
                        yOffset + 1.0 * node.wristAngle - targetHeight));
            }

            this.setSize(
                (float)(2.0f * mapWidth),
                (float)(2.0f * mapHeight));
        }

        @Override
        public void act(float delta)
        {
            super.act(delta);
        }

        /**
         * Draw a frame of animation based on the current state of the simulation.
         * Remember that (0, 0) is at the bottom left!
         */
        @Override
        public void draw(Batch batch, float parentAlpha)
        {
            super.draw(batch, parentAlpha);

            batch.setColor(1f, 1f, 1f, parentAlpha);
            batch.draw(this.currentPixMapTexture, this.getX(), this.getY(), this.getWidth(), this.getHeight());

            for (Point2d point : this.targetPoints)
            {
                batch.draw(
                    this.target,
                    this.getX() + (float)point.x * 2.0f,
                    this.getY() + (float)point.y * 2.0f,
                    this.target.getWidth() * 2.0f,
                    this.target.getHeight() * 2.0f);
            }
        }

        @Override
        public void dispose()
        {
            this.currentPixMapTexture.dispose();
            this.pixMap.dispose();
        }
    }
}
