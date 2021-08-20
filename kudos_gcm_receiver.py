from __future__ import unicode_literals, print_function
import socket
import time
import logging
import argparse
import sys
import rospy
from darkneta.msg import gcm

from construct import Container, ConstError
from gamestate import GameState, ReturnData, GAME_CONTROLLER_RESPONSE_VERSION

logger = logging.getLogger('game_controller')
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
logger.addHandler(console_handler)

DEFAULT_LISTENING_HOST = '0.0.0.0'
GAME_CONTROLLER_LISTEN_PORT = 3838
GAME_CONTROLLER_ANSWER_PORT = 3939

parser = argparse.ArgumentParser()
parser.add_argument('--team', type=int, default=1, help="team ID, default is 1")
parser.add_argument('--player', type=int, default=1, help="player ID, default is 1")
parser.add_argument('--goalkeeper', action="store_true", help="if this flag is present, the player takes the role of the goalkeeper")

class priROS():
    def __init__(self):
        pass



class GameStateReceiver(object):
    def __init__(self, team, player, is_goalkeeper, addr=(DEFAULT_LISTENING_HOST, GAME_CONTROLLER_LISTEN_PORT), answer_port=GAME_CONTROLLER_ANSWER_PORT):
        self.team = team
        self.player = player
        self.man_penalize = True
        self.is_goalkeeper = is_goalkeeper
        self.addr = addr
        self.answer_port = answer_port
        self.state = None
        self.time = None
        self.socket = None
        self.running = True
        self._open_socket()

    def _open_socket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.addr)
        self.socket.settimeout(0.5)
        self.socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def receive_forever(self):
        while self.running:
            try:
                self.receive_once()
            except IOError as e:
                logger.debug("Fehler beim Senden des KeepAlive: " + str(e))

    def receive_once(self):
        try:
            data, peer = self.socket.recvfrom(GameState.sizeof())
            print("data_len:", len(data))
            self.state = GameState.parse(data)
            self.time = time.time()
            self.on_new_gamestate(self.state)
            self.answer_to_gamecontroller(peer)

        except AssertionError as ae:
            logger.error(ae.message)
        except socket.timeout:
            logger.warning("Socket timeout")
        except ConstError:
            logger.warning("Parse Error: Probably using an old protocol!")
        except Exception as e:
            logger.exception(e)
            pass

    def answer_to_gamecontroller(self, peer):
        return_message = 0
        if self.man_penalize:
            return_message = 0
        else:
            return_message = 2
        if self.is_goalkeeper:
            return_message = 3

        data = Container(header=b"RGrt", version=GAME_CONTROLLER_RESPONSE_VERSION, team=self.team, player=self.player, message=return_message)
        try:
            destination = peer[0], GAME_CONTROLLER_ANSWER_PORT
            self.socket.sendto(ReturnData.build(data), destination)
        except Exception as e:
            logger.log("Network Error: %s" % str(e))

    def on_new_gamestate(self, state):
        raise NotImplementedError()

    def get_last_state(self):
        return self.state, self.time

    def get_time_since_last_package(self):
        return time.time() - self.time

    def stop(self):
        self.running = False

    def set_manual_penalty(self, flag):
        self.man_penalize = flag


class SampleGameStateReceiver(GameStateReceiver):
    def talker(self, message_form):
        pub = rospy.Publisher('visionGcm', gcm, queue_size=1)
        rospy.init_node('visionGcm', anonymous = False)
        message = gcm()
        message.main_game_state = message_form['game_state']
        rospy.loginfo(message)
        pub.publish(message)
    def on_new_gamestate(self, state):
        message_form = {
            'game_state':-1}
        print(state.game_state)
        if state.game_state == 'STATE_INITIAL':
            message_form['game_state'] = 0
        elif state.game_state == 'STATE_READY':
            message_form['game_state'] = 1
        elif state.game_state == 'STATE_SET':
            message_form['game_state'] = 2
        elif state.game_state == 'STATE_PLAYING':
            message_form['game_state'] = 3
        elif state.game_state == 'STATE_FINISHED':
            message_form['game_state'] = 0
        self.talker(message_form)
        
        #write_ros_message_here
        #
        #print(state.teams[0].team_number)

if __name__ == '__main__':
    args = parser.parse_args(sys.argv[1:])
    rec = SampleGameStateReceiver(team=args.team, player=args.player, is_goalkeeper=args.goalkeeper)
    rec.receive_forever()
