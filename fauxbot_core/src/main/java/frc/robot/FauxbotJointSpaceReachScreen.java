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
import com.badlogic.gdx.scenes.scene2d.Group;
import com.badlogic.gdx.scenes.scene2d.InputEvent;
import com.badlogic.gdx.scenes.scene2d.InputListener;
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
import frc.lib.robotprovider.FauxbotJoystick;
import frc.lib.robotprovider.Point2d;
import frc.robot.mechanisms.ArmKinematicsCalculator;
import frc.robot.mechanisms.ArmKinematicsCalculator.ExtensionType;
import frc.robot.simulation.RobotSimulator;
import frc.robot.simulation.SimulatorBase;
import space.earlygrey.shapedrawer.ShapeDrawer;

public class FauxbotJointSpaceReachScreen implements Screen
{
    private static final float SCALE = 2.0f;
    private final FauxbotGame game;

    private final Stage stage;
    private final Skin skin;
    private final JointSpaceDiagram jointSpaceDiagram;
    private final Label informationLabel;
    private final Label pointLabel;
    private final Label intakeTopLocationLabel;
    private final Label intakeBottomLocationLabel;
    private final Label shooterTopLocationLabel;
    private final Label shooterBottomLocationLabel;

    public FauxbotJointSpaceReachScreen(final FauxbotGame game)
    {
        this.game = game;

        this.stage = new Stage(new ExtendViewport(900, 750));
        Gdx.input.setInputProcessor(this.stage);

        this.skin = new Skin(Gdx.files.internal("skin/irs1318skin.json"));

        Table primaryTable = new Table(this.skin);
        primaryTable.setFillParent(true);
        // primaryTable.setDebug(true);

        Label title = new Label("JointSpace Reach diagram", this.skin, "title");
        primaryTable.add(title).pad(20).top().colspan(2).row();

        Table jsdScrollTable = new Table(this.skin);
        ScrollPane jsdScrollPane = new ScrollPane(jsdScrollTable, this.skin);
        // jsdScrollTable.setDebug(true);

        Table dataScrollTable = new Table(this.skin);
        ScrollPane dataScrollPane = new ScrollPane(dataScrollTable, this.skin);
        // dataScrollTable.setDebug(true);

        Label anglesLabel = new Label("Angles:", this.skin);
        dataScrollTable.add(anglesLabel).pad(5).left();
        this.pointLabel = new Label("", this.skin);
        dataScrollTable.add(this.pointLabel).expandX().row();

        Label nodeLabel = new Label("Node:", this.skin);
        dataScrollTable.add(nodeLabel).pad(5).left();
        this.informationLabel = new Label("", this.skin);
        dataScrollTable.add(this.informationLabel).expandX().row();

        Label intakeBottomLabel = new Label("Intake Bottom:", this.skin);
        dataScrollTable.add(intakeBottomLabel).pad(5).left();
        this.intakeBottomLocationLabel = new Label("", this.skin);
        dataScrollTable.add(this.intakeBottomLocationLabel).expandX().row();

        Label intakeTopLabel = new Label("Intake Top:", this.skin);
        dataScrollTable.add(intakeTopLabel).pad(5).left();
        this.intakeTopLocationLabel = new Label("", this.skin);
        dataScrollTable.add(this.intakeTopLocationLabel).expandX().row();

        Label shooterBottomLabel = new Label("Shooter Bottom:", this.skin);
        dataScrollTable.add(shooterBottomLabel).pad(5).left();
        this.shooterBottomLocationLabel = new Label("", this.skin);
        dataScrollTable.add(this.shooterBottomLocationLabel).expandX().row();

        Label shooterTopLabel = new Label("Shooter Top:", this.skin);
        dataScrollTable.add(shooterTopLabel).pad(5).left();
        this.shooterTopLocationLabel = new Label("", this.skin);
        dataScrollTable.add(this.shooterTopLocationLabel).expandX().row();

        this.jointSpaceDiagram = new JointSpaceDiagram();
        jsdScrollTable.add(this.jointSpaceDiagram).center().expand().colspan(2).row();

        SplitPane splitPane = new SplitPane(dataScrollPane, jsdScrollPane, false, this.skin);
        primaryTable.add(splitPane).expand().fill().pad(5);

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

    public class JointSpaceDiagram extends Group implements Disposable
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

        private Point2d[][][] angleToPointsMap;
        private Pixmap pixMap;
        private Texture currentPixMapTexture;

        public JointSpaceDiagram()
        {
            double mapWidth = 2.0 * (Math.round(TuningConstants.ARM_SHOULDER_MAX_POSITION) - Math.round(TuningConstants.ARM_SHOULDER_MIN_POSITION));
            double mapHeight = Math.round(TuningConstants.ARM_WRIST_MAX_POSITION) - Math.round(TuningConstants.ARM_WRIST_MIN_POSITION);

            // reminder: pixMap is top-left origin, whereas most other libGDX stuff is bottom-left origin
            this.pixMap = new Pixmap((int)mapWidth, (int)mapHeight, Format.RGB888);
            this.pixMap.setColor(Color.WHITE);
            this.pixMap.fill();
            this.currentPixMapTexture = new Texture(this.pixMap);

            Color currentColor = Color.GREEN;
            this.pixMap.setColor(currentColor);

            this.angleToPointsMap = new Point2d[(int)mapWidth][(int)mapHeight][5];

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

                    int xIndex = (int)(xOffset + shoulderPosition * 2.0);
                    int yIndex = (int)(yOffset + wristPosition);
                    this.pixMap.drawPixel((int)(xOffset + shoulderPosition * 2.0), (int)(mapHeight - (yOffset + wristPosition)));
                    this.angleToPointsMap[xIndex][yIndex][0] = calculator.getIntakeBottomAbsPos();
                    this.angleToPointsMap[xIndex][yIndex][1] = calculator.getIntakeTopAbsPos();
                    this.angleToPointsMap[xIndex][yIndex][2] = calculator.getShooterBottomAbsPos();
                    this.angleToPointsMap[xIndex][yIndex][3] = calculator.getShooterTopAbsPos();
                    this.angleToPointsMap[xIndex][yIndex][4] = new Point2d(shoulderPosition, wristPosition);
                }
            }

            this.currentPixMapTexture.draw(this.pixMap, 0, 0);

            Texture targetTexture = new Texture(Gdx.files.internal("images/target.png"));

            Set<ArmKinematicsCalculator.ArmGraphNode> allNodes = ArmKinematicsCalculator.getAllGraphNodes();
            int targetHalfWidth = (int)(targetTexture.getWidth() / 2.0);
            int targetHalfHeight = (int)(targetTexture.getHeight() / 2.0);
            for (ArmKinematicsCalculator.ArmGraphNode node : allNodes)
            {
                this.addActor(
                    new Target(
                        node.name,
                        new Point2d(
                            xOffset + 2.0 * node.shoulderAngle - targetHalfWidth,
                            yOffset + 1.0 * node.wristAngle - targetHalfHeight),
                        targetTexture));
            }

            this.setSize(
                (float)(FauxbotJointSpaceReachScreen.SCALE * mapWidth),
                (float)(FauxbotJointSpaceReachScreen.SCALE * mapHeight));

            addListener(
                new InputListener()
                {
                    /** Called any time the mouse cursor or a finger touch is moved over an actor. On the desktop, this event occurs even when no
                     * mouse buttons are pressed (pointer will be -1).
                     * @param fromActor May be null.
                     * @see InputEvent */
                    public void enter(InputEvent event, float x, float y, int pointer, Actor fromActor)
                    {
                        if (x >= 0 && x <= getWidth() && y >= 0 && y <= getHeight())
                        {
                            double shoulderAngle = x / 4.0 + Math.round(TuningConstants.ARM_SHOULDER_MIN_POSITION);
                            double wristAngle = y / 2.0 + Math.round(TuningConstants.ARM_WRIST_MIN_POSITION);
                            pointLabel.setText(
                                String.format(
                                    "%.2f, %.2f",
                                    shoulderAngle,
                                    wristAngle));

                            int xIndex = (int)(x / 2.0);
                            int yIndex = (int)(y / 2.0);
                            intakeBottomLocationLabel.setText(angleToPointsMap[xIndex][yIndex][0].toString());
                            intakeTopLocationLabel.setText(angleToPointsMap[xIndex][yIndex][1].toString());
                            shooterBottomLocationLabel.setText(angleToPointsMap[xIndex][yIndex][2].toString());
                            shooterTopLocationLabel.setText(angleToPointsMap[xIndex][yIndex][3].toString());

                            if (Math.abs(angleToPointsMap[xIndex][yIndex][4].x - shoulderAngle) > 1.0 ||
                                Math.abs(angleToPointsMap[xIndex][yIndex][4].y - wristAngle) > 1.0)
                            {
                                System.out.println("Error in angleToPointsMap indexing: " + angleToPointsMap[xIndex][yIndex][4].toString() + " vs (" + shoulderAngle + ", " + wristAngle + ")");
                            }
                        }
                    }

                    /** Called any time the mouse is moved when a button is not down. This event only occurs on the desktop. When true is returned,
                     * the event is {@link Event#handle() handled}.
                     * @see InputEvent */
                    public boolean mouseMoved(InputEvent event, float x, float y)
                    {
                        if (x >= 0 && x <= getWidth() && y >= 0 && y <= getHeight())
                        {
                            double shoulderAngle = x / 4.0 + Math.round(TuningConstants.ARM_SHOULDER_MIN_POSITION);
                            double wristAngle = y / 2.0 + Math.round(TuningConstants.ARM_WRIST_MIN_POSITION);
                            pointLabel.setText(
                                String.format(
                                    "%.2f, %.2f",
                                    shoulderAngle,
                                    wristAngle));

                            int xIndex = (int)(x / 2.0);
                            int yIndex = (int)(y / 2.0);
                            intakeBottomLocationLabel.setText(angleToPointsMap[xIndex][yIndex][0].toString());
                            intakeTopLocationLabel.setText(angleToPointsMap[xIndex][yIndex][1].toString());
                            shooterBottomLocationLabel.setText(angleToPointsMap[xIndex][yIndex][2].toString());
                            shooterTopLocationLabel.setText(angleToPointsMap[xIndex][yIndex][3].toString());

                            if (Math.abs(angleToPointsMap[xIndex][yIndex][4].x - shoulderAngle) > 1.0 ||
                                Math.abs(angleToPointsMap[xIndex][yIndex][4].y - wristAngle) > 1.0)
                            {
                                System.out.println("Error in angleToPointsMap indexing: " + angleToPointsMap[xIndex][yIndex][4].toString() + " vs (" + shoulderAngle + ", " + wristAngle + ")");
                            }
                        }

                        return false;
                    }

                    /** Called any time the mouse cursor or a finger touch is moved out of an actor. On the desktop, this event occurs even when no
                     * mouse buttons are pressed (pointer will be -1).
                     * @param toActor May be null.
                     * @see InputEvent */
                    public void exit(InputEvent event, float x, float y, int pointer, Actor toActor)
                    {
                        if (x < 0 || x > getWidth() || y < 0 || y > getHeight())
                        {
                            pointLabel.setText("");
                            intakeBottomLocationLabel.setText("");
                            intakeTopLocationLabel.setText("");
                            shooterBottomLocationLabel.setText("");
                            shooterTopLocationLabel.setText("");
                        }
                    }
                });
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
            batch.setColor(1f, 1f, 1f, parentAlpha);
            batch.draw(this.currentPixMapTexture, this.getX(), this.getY(), this.getWidth(), this.getHeight());

            super.draw(batch, parentAlpha);
        }

        @Override
        public void dispose()
        {
            this.currentPixMapTexture.dispose();
            this.pixMap.dispose();
        }

        private class Target extends Actor
        {
            private final String name;
            private final Point2d location;
            private final Texture targetTexture;

            public Target(String name, Point2d location, Texture targetTexture)
            {
                this.targetTexture = targetTexture;

                this.location = location;
                this.name = name;
                this.setSize(
                    this.targetTexture.getWidth() * FauxbotJointSpaceReachScreen.SCALE,
                    this.targetTexture.getHeight() * FauxbotJointSpaceReachScreen.SCALE);
                this.setX((float)this.location.x * FauxbotJointSpaceReachScreen.SCALE);
                this.setY((float)this.location.y * FauxbotJointSpaceReachScreen.SCALE);

                addListener(
                    new InputListener()
                    {
                        /** Called any time the mouse cursor or a finger touch is moved over an actor. On the desktop, this event occurs even when no
                         * mouse buttons are pressed (pointer will be -1).
                         * @param fromActor May be null.
                         * @see InputEvent */
                        public void enter(InputEvent event, float x, float y, int pointer, Actor fromActor)
                        {
                            informationLabel.setText(name);
                        }

                        /** Called any time the mouse cursor or a finger touch is moved out of an actor. On the desktop, this event occurs even when no
                         * mouse buttons are pressed (pointer will be -1).
                         * @param toActor May be null.
                         * @see InputEvent */
                        public void exit(InputEvent event, float x, float y, int pointer, Actor toActor)
                        {
                            informationLabel.setText("");
                        }
                    });
            }

            @Override
            public void act(float delta)
            {
                super.act(delta);
            }

            @Override
            public void draw(Batch batch, float parentAlpha)
            {
                super.draw(batch, parentAlpha);

                batch.draw(
                    this.targetTexture,
                    this.getX(),
                    this.getY(),
                    this.getWidth(),
                    this.getHeight());
            }
        }
    }
}
