from .elements import *

class Field:
    def __init__(self, side):
        self.width = 2.2
        self.height = 1.8
        self.goalAreaWidth = 0.5
        self.goalAreaHeight = 0.05

        self.xmargin = 0.30
        self.ymargin = 0.18
        self.side = side

        self.goalDepth = 0.15


        self.areaEllipseSize = (0.28, 0.35)
        self.areaEllipseCenter = (-self.maxX + 0.1, 0)

    @property
    def maxX(self):
        return self.width / 2

    @property
    def maxY(self):
        return self.height / 2

    @property
    def size(self):
        return (self.maxX, self.maxY)

    @property
    def marginX(self):
        return self.maxX - self.xmargin
    
    @property
    def marginY(self):
        return self.maxY - self.ymargin

    @property
    def marginPos(self):
        return (self.marginX, self.marginY)

    @property
    def goalPos(self):
        return (self.maxX, 0)

    @property
    def goalAreaSize(self):
        return (self.goalAreaWidth, self.goalAreaHeight)

class World:
    def __init__(self, n_robots=[0,1,2,3,4], side=1, team_yellow=False, immediate_start=False, referee=False, firasim=False, debug=False, mirror=False, control=False, last_command=None, i = False):
        self.n_robots = n_robots
        self._team = [None,None,None,None,None]
        self.enemies = [None,None,None,None,None]
        self.staticen = False
        for i in self.n_robots:
            self._team[i] = TeamRobot(self, i, on=immediate_start)
        for i in self.n_robots:
            self.enemies[i] = TeamRobot(self, i, on=immediate_start)
        self.ball = Ball(self)
        self.field = Field(side)
        self.referee = referee
        self.firasim = firasim
        self.debug = debug
        self.mirror = mirror
        self.control =  control
        self.last_command = last_command
        self._referenceTime = 0
        self.dt = 0
        
        self.team_yellow = team_yellow

        self.allyGoals = 0
        self.enemyGoals = 0
        self.updateCount = 0
                    
        
    def FIRASim_update(self, message):
        # teamPos = zip(message["ally_x"], message["ally_y"], message["ally_th"], message["ally_vx"], message["ally_vy"], message["ally_w"])
        # enemiesPos = zip(message["enemy_x"], message["enemy_y"], message["enemy_th"], message["enemy_vx"], message["enemy_vy"], message["enemy_w"])
        if self.debug:
                print("-------------------------")
                print("Executando com firasim:")
                if self.mirror: 
                    print("UTILIZANDO CAMPO INVERTIDO")
                else:                   
                    print("UTILIZANDO CAMPO SEM INVERSÃO")

        if self.team_yellow: 
            yellow = self.team
            blue = self.enemies
        else:
            blue = self.team
            yellow = self.enemies
            
        if self.team_yellow:
            for i, robot in enumerate(message.frame.robots_yellow):
                if i < len(self.n_robots):
                    #yellow[robot_id].update(message.robots_yellow[robot_id].x,message.robots_yellow[robot_id].y, message.robots_yellow[robot_id].orientation)
                    if self.debug:
                        print(f"Yellow - {i} | x {robot.x:.3f} | y {robot.y:.3f} | th {robot.orientation:.3f} | vx {robot.vx:.3f} | vy {robot.vy:.3f} | vorientation {robot.vorientation:.3f}")
                    yellow[self.n_robots[i]].update_FIRASim(robot.x, robot.y, robot.orientation, robot.vx, robot.vy, robot.vorientation)
            for i, robot in enumerate(message.frame.robots_blue):
                if i < len(self.n_robots):
                    #yellow[robot_id].update(message.robots_yellow[robot_id].x,message.robots_yellow[robot_id].y, message.robots_yellow[robot_id].orientation)
                    if self.debug:
                        print(f"Blue - {i} | x {robot.x:.3f} | y {robot.y:.3f} | th {robot.orientation:.3f} | vx {robot.vx:.3f} | vy {robot.vy:.3f} | vorientation {robot.vorientation:.3f}")
                    blue[self.n_robots[i]].update_FIRASim(robot.x, robot.y, robot.orientation, robot.vx, robot.vy, robot.vorientation)


        else:
            for i, robot in enumerate(message.frame.robots_blue):
                if i < len(self.n_robots):
                    if self.debug:
                        print(f"Blue - {i} | x {robot.x:.3f} | y {robot.y:.3f} | th {robot.orientation:.3f} | vx {robot.vx:.3f} | vy {robot.vy:.3f} | vorientation {robot.vorientation:.3f}")
                    blue[self.n_robots[i]].update_FIRASim(robot.x, robot.y, robot.orientation, robot.vx, robot.vy, robot.vorientation)
            for i, robot in enumerate(message.frame.robots_yellow):
                if i < len(self.n_robots):
                        #yellow[robot_id].update(message.robots_yellow[robot_id].x,message.robots_yellow[robot_id].y, message.robots_yellow[robot_id].orientation)
                        if self.debug:
                            print(f"Yellow - {i} | x {robot.x:.3f} | y {robot.y:.3f} | th {robot.orientation:.3f} | vx {robot.vx:.3f} | vy {robot.vy:.3f} | vorientation {robot.vorientation:.3f}")
                        yellow[self.n_robots[i]].update_FIRASim(robot.x, robot.y, robot.orientation, robot.vx, robot.vy, robot.vorientation)

        # for robot, pos in zip(self.team, teamPos): robot.update(*pos)
        # for robot, pos in zip(self.enemies, enemiesPos): robot.update(*pos)
        #self.ball.update(message["ball_x"], message["ball_y"], message["ball_vx"], message["ball_vy"])
        if self.debug:
            print(f"BALL | x {(message.frame.ball.x):.2f} | y {(message.frame.ball.y):.2f}")
        self.ball.update_element_FIRASim(message.frame.ball.x, message.frame.ball.y, message.frame.ball.vx, message.frame.ball.vy)

        self.updateCount += 1

    def setLastCommand(self, last_command):
        self.last_command = last_command

    def addAllyGoal(self):
        print("Gol aliado!")
        self.allyGoals += 1

    def addEnemyGoal(self):
        print("Gol inimigo!")
        self.enemyGoals += 1

    @property
    def goals(self):
        return self.allyGoals + self.enemyGoals

    @property
    def balance(self):
        return self.allyGoals - self.enemyGoals

    @property
    def team(self):
        return self._team#[robot for robot in self._team if robot.on]

    @property
    def raw_team(self):
        return self._team