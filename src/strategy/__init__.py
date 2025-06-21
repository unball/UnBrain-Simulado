from abc import ABC, abstractmethod
from .entity.attacker import Attacker
from .entity.goalKeeper import GoalKeeper
from .entity.defender import Defender
from .entity.midfielder import Midfielder
from .entity.SecAttacker import SecAttacker
from .entity.controlTester import ControlTester
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

    def getEntity(self, entity):
        return entity.__class__.__name__
    
    def manageReferee(self, command):
        if command is None: 
            for robot in self.world.raw_team: 
                if robot is not None:
                    robot.turnOff()
        
        else:

            ComandoReferee = {
                Foul.KICKOFF:"Começo de jogo",
                Foul.FREE_BALL:"Free ball",
                Foul.PENALTY_KICK:"Pênaulti",
                Foul.GOAL_KICK:"Tiro de meta",
                Foul.GAME_ON:"Start",
                Foul.STOP:"Stop",
                Foul.HALT:"Stop"
            }

            if self.world.debug:
                print(f'{ComandoReferee[command.foul]} no quadrante {command.foulQuadrant}')
            [robot.turnOn() for robot in self.world.team if robot is not None and ComandoReferee[command.foul] == "Start"] + [robot.turnOff() for robot in self.world.team if robot is not None and ComandoReferee[command.foul] != "Start"]
                    
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

        if self.world.ball.pos[0] < -0.71:
            return [GoalKeeper, Attacker, Defender, Defender, Attacker]
        else:
            return [GoalKeeper,Attacker, Defender, Defender, Attacker]
    #alteramos para que ToDecide (a variável que instancia esta função) esteja em formato de lista e não em um np.ndarray
    def availableRobotIndexes(self):
        return self.world.n_robots.copy()

    def DecideBestAttacker(self, formation, toDecide, hasMaster):
        d = []
        for i in toDecide:
            if i != self.currentAttacker:
                d += [norm(self.world.team[i].pos, self.world.ball.pos)]

        self.currentAttacker = bestWithHyst(self.currentAttacker, toDecide, d, 0.20)
        if self.world.team[self.currentAttacker].entity != None and self.getEntity(self.currentAttacker) == 'Attacker':
            if self.world.team[self.currentAttacker].entity.slave == False:
                self.world.team[self.currentAttacker].updateEntity(Attacker, ballShift=0, slave=False)
            elif self.world.team[self.currentAttacker].entity.slave == True:
                self.world.team[self.currentAttacker].updateEntity(Attacker, ballShift=0, slave=True)
        else:
            self.world.team[self.currentAttacker].updateEntity(Attacker, ballShift=0, slave=False)
        if self.currentAttacker in toDecide:
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
        d = []
        for i in toDecide:
            if i != self.currentDefender:
                d += [norm(self.world.team[i].pos, target)]

        self.currentDefender = bestWithHyst(self.currentDefender, toDecide, d, 0.20)
        self.world.team[self.currentDefender].updateEntity(Defender)

        if self.currentDefender in toDecide:
            toDecide.remove(self.currentDefender)
        formation.remove(Defender)

        return formation, toDecide

    def update(self, world):
        #Como estamos trabalhando a partir de um número dado de quantos robôs temos, é melhor tratar esses updates em um ciclo
        #De repetição que tem range máximo o número de robôs e atualizaremos com base na prioridade (goleiro primeiro, atacante segundo) 
        #obs: (ficará comentado o que era antes)
        if self.static_entities:
            roles=[Attacker, SecAttacker, Defender, Defender, GoalKeeper]
            if self.world.staticen is False:
                for robo in self.world.n_robots:
                    self.world.team[int(robo)].updateEntity(roles[int(robo)])
                    self.world.staticen = True
        elif self.world.control_tester:
            for i in self.world.n_robots:
                self.world.team[i].updateEntity(ControlTester)
        else:
            formation = self.formationDecider()
            toDecide = self.availableRobotIndexes()

            if GoalKeeper in formation and len(toDecide) >= 1:
                formation, toDecide = self.decideBestGoalKeeper(formation, toDecide)
            
            hasMaster = False
            if Attacker in formation and len(toDecide) >= 1:
                formation, toDecide = self.DecideBestAttacker(formation, toDecide, hasMaster)
                hasMaster = True

            if Defender in formation and len(toDecide) >= 1:
                formation, toDecide = self.decideBestDefender(formation, toDecide)          

            if Attacker in formation and len(toDecide) >= 1:
                formation, toDecide = self.DecideBestAttacker(formation,toDecide, hasMaster)

            if Defender in formation and len(toDecide) >= 1:
                formation, toDecide = self.decideBestDefender(formation, toDecide)
            
            if Defender in formation and len(toDecide) >= 1:
                formation, toDecide = self.decideBestDefender(formation, toDecide)
        for robot in self.world.team:
            if robot is not None:
                robot.updateSpin()
                if robot.entity is not None:
                    robot.entity.fieldDecider()
                    robot.entity.directionDecider()