from abc import ABC, abstractmethod
from .entity.attacker import Attacker
from .entity.goalKeeper import GoalKeeper
from .entity.defender import Defender
from .entity.midfielder import Midfielder
from .entity.controlTest import ControlTester
from client.protobuf.vssref_common_pb2 import Foul, Quadrant
from client.referee import RefereeCommands
from tools import sats, norml, unit, angl, angError, projectLine, howFrontBall, norm, bestWithHyst
from .movements import blockBallElipse
from copy import copy
import numpy as np
import time

class Strategy(ABC):
    def __init__(self, world):
        super().__init__()

        self.world = world

    @abstractmethod
    def manageReferee(self, rp, command):
        pass

    @abstractmethod
    def update(self):
        pass

class MainStrategy(Strategy):
    def __init__(self, world, static_entities=False):
        super().__init__(world)

        # States
        self.currentAttacker = None
        self.currentDefender = None

        # Variables
        self.static_entities = static_entities

    def manageReferee(self, command):
        #código do arp descontinuado, se for preciso para futuras alterações, revisar versões antigas        
        # Pegar apenas id que existe dos robos
        robot_id = []
        for robot in self.world.team:
            if robot is None:
                return
        for robot in self.world.team:
            if robot is not None:
                robot_id.append(robot.id)

        if command is None: 
            for robot in self.world.raw_team: 
                if robot is not None:
                    robot.turnOff()
        
        else:
            self.goalkeeperIndx = None
            self.AttackerIdx = None
                        # Inicia jogo   
            if command.foul == Foul.GAME_ON:
                
                if(self.world.debug):
                    print("COMANDO START ENVIADO")
                
                for robot in self.world.raw_team:
                    if robot is not None:
                        robot.turnOn()
                        
            elif command.foul == Foul.STOP or command.foul == Foul.HALT:
                
                if(self.world.debug):
                    print("COMANDO STOP OU HALT ENVIADO")
                
                for robot in self.world.raw_team: 
                    if robot is not None:
                        robot.turnOff()
            
            if command.foul == Foul.KICKOFF:
                for robot in self.world.raw_team: 
                    if robot is not None:
                        robot.turnOff()

            elif command.foul == Foul.PENALTY_KICK:
                
                for robot in self.world.raw_team: 
                    if robot is not None:
                        robot.turnOff()                 
            
            elif command.foul == Foul.FREE_BALL and command.foulQuadrant == Quadrant.QUADRANT_1:
                if(self.world.debug):
                    print("FREE BALL Q1")
                for robot in self.world.raw_team: 
                    if robot is not None:
                        robot.turnOff()

            
            elif command.foul == Foul.FREE_BALL and command.foulQuadrant == Quadrant.QUADRANT_2:
                
                if(self.world.debug):
                    print("FREE BALL Q2")
                for robot in self.world.raw_team: 
                    if robot is not None:
                        robot.turnOff()

            elif command.foul == Foul.FREE_BALL and command.foulQuadrant == Quadrant.QUADRANT_3:
                
                if(self.world.debug):
                    print("FREE BALL Q3")

                for robot in self.world.raw_team: 
                    if robot is not None:
                        robot.turnOff()

            elif command.foul == Foul.FREE_BALL and command.foulQuadrant == Quadrant.QUADRANT_4:
                
                if(self.world.debug):
                    print("FREE BALL Q4")
                    
                for robot in self.world.raw_team: 
                    if robot is not None:
                        robot.turnOff()

            elif command.foul == Foul.GOAL_KICK:
                
                if(self.world.debug):
                    print("GOAL KICK")
                for robot in self.world.raw_team: 
                    if robot is not None:
                        robot.turnOff()        
                    
    
    def nearestGoal(self, indexes):
        rg = np.array([-1.1, 0])
        rrs = np.array([self.world.team[i].pos for i in indexes])
        nearest = indexes[np.argmin(np.linalg.norm(rrs-rg, axis=1))]

        return nearest

    def ellipseTarget(self):
        rb = np.array(self.world.ball.pos)
        vb = np.array(self.world.ball.v)
        rr = np.array([0,0,0]) # dummy, usado para computar angulo do pose, não é necessário aqui
        
        pose, spin = blockBallElipse(rb, vb, rr, self.world.field.areaEllipseCenter, *self.world.field.areaEllipseSize)

        return pose[:2]

    def formationDecider(self):
        if self.world.ball.pos[0] < 0.35 and self.world.team_yellow is True:
            return [GoalKeeper, Defender, Defender, Defender, Attacker]
        else:
            return [GoalKeeper,Attacker, Defender, Defender, Attacker]
    #alteramos para que ToDecide (a variável que instancia esta função) esteja em formato de lista e não em um np.ndarray
    def availableRobotIndexes(self):
        return self.world.n_robots.copy()

    def DecideBestAttacker(self, formation, toDecide, hasMaster):
        distances = [norm(self.world.ball.pos, self.world.team[robotIndex].pos) for robotIndex in toDecide]
        self.currentAttacker = bestWithHyst(self.currentAttacker, toDecide, distances, 0.2)
        self.world.team[self.currentAttacker].updateEntity(Attacker, ballShift=0, slave=hasMaster)
        toDecide.remove(self.currentAttacker)
        formation.remove(Attacker)
        return formation, toDecide

    def decideBestGoalKeeper(self, formation, toDecide):
        nearest = self.nearestGoal(toDecide)
        self.world.team[nearest].updateEntity(GoalKeeper)
        
        toDecide.remove(nearest)
        formation.remove(GoalKeeper)
        
        return formation, toDecide

    def decideBestDefender(self, formation, toDecide):
        target = self.ellipseTarget()
        distances = [norm(target, self.world.team[robotIndex].pos) for robotIndex in toDecide]

        self.currentDefender = bestWithHyst(self.currentDefender, toDecide, distances, 0.20)
        self.world.team[self.currentDefender].updateEntity(Defender)

        toDecide.remove(self.currentDefender)
        formation.remove(Defender)

        return formation, toDecide

    def update(self, world):
        #Como estamos trabalhando a partir de um número dado de quantos robôs temos, é melhor tratar esses updates em um ciclo
        #De repetição que tem range máximo o número de robôs e atualizaremos com base na prioridade (goleiro primeiro, atacante segundo) 
        #obs: (ficará comentado o que era antes)
        if self.static_entities:
            roles=[Attacker, GoalKeeper, Midfielder, Midfielder, Attacker]
            if self.world.staticen is False:
                for robo in self.world.n_robots:
                    self.world.team[int(robo)].updateEntity(roles[int(robo)])
                    self.world.staticen = True
            #self.world.team[0].updateEntity(Attacker)
            #self.world.team[1].updateEntity(Defender)
            #self.world.team[2].updateEntity(GoalKeeper)

        #mesma coisa aqui só que sem o static-entities
        # elif world.control:
        #     for i in self.world.n_robots:
        #         self.world.team[i].updateEntity(ControlTester, forced_update=True)
        #     #self.world.team[0].updateEntity(ControlTester, forced_update=True)
        #     #self.world.team[1].updateEntity(ControlTester, forced_update=True)
        #     #self.world.team[2].updateEntity(ControlTester, forced_update=True)

        else:
            
            formation = self.formationDecider()
            toDecide = self.availableRobotIndexes()


            if GoalKeeper in formation and len(toDecide) >= 1:
                formation, toDecide = self.decideBestGoalKeeper(formation, toDecide)

            hasMaster = False
            if Attacker in formation and len(toDecide) >= 1:
                formation, toDecide = self.DecideBestAttacker(formation, toDecide, hasMaster)
                hasMaster = True

            if Attacker in formation and len(toDecide) >= 1:
                formation, toDecide = self.DecideBestAttacker(formation,toDecide, hasMaster)

            if Defender in formation and len(toDecide) >= 1:
                formation, toDecide = self.decideBestDefender(formation, toDecide)

            if Defender in formation and len(toDecide) >= 1:
                formation, toDecide = self.decideBestDefender(formation, toDecide)
            
            if Defender in formation and len(toDecide) >= 1:
                formation, toDecide = self.decideBestDefender(formation, toDecide)

            if Midfielder in formation and len(toDecide) >= 1:
                self.world.team[toDecide[0]].updateEntity(Midfielder)
                toDecide.remove(toDecide[0])
                formation.remove(Midfielder)
        for robot in self.world.team:
            if robot is not None:
                robot.updateSpin()
                if robot.entity is not None:
                    robot.entity.fieldDecider()
                    robot.entity.directionDecider()