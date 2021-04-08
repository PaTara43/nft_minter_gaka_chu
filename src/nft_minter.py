#!/usr/bin/python3

import json
import logging
import rospy
import rospkg
import yaml

from pinatapy import PinataPy
from std_msgs.msg import String
from web3 import Web3
from web3.gas_strategies.time_based import medium_gas_price_strategy


def read_configuration(dirname) -> dict:

    config_path = dirname + '/config/config.yaml'
    logging.debug(config_path)

    try:
        with open(config_path) as f:
            content = f.read()
            config = yaml.load(content, Loader=yaml.FullLoader)
            logging.debug(f"Configuration dict: {content}")
            return config
    except Exception as e:
        while True:
            logging.error("Error in configuration file!")
            logging.error(e)
            exit()

class Error(Exception):
    pass


def _pin_to_pinata(filename):

    if pinata_api and pinata_secret_api:
        pinata = PinataPy(pinata_api, pinata_secret_api)
        pinata.pin_file_to_ipfs(filename)
        rospy.loginfo("File sent")
        return pinata.pin_list()['rows'][0]['ipfs_pin_hash']


def callback(data, packagePath):

    #process files
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    vid_qm = data.data.split()[0]
    rosbag_qm = data.data.split()[1]
    pic_qm = data.data.split()[2]
    name = data.data.split()[3]

    #json metadata
    description = 'This is a sample description for a NFT minted by robot.'

    metadata_d = {
      "description": description,
      "external_url": "https://dapp.robonomics.network",
      "image": "ipfs://" + pic_qm,
      "name": name.replace("_", " ") + ". Artwork by robot-artist Gaka-Сhu",
      "animation_url": "ipfs://" + vid_qm,
    }
    metadata = json.dumps(metadata_d, indent=2)

    if testnet:
        with open(packagePath + "/data/metadata_" + name + "_testnet.json", 'w') as metadata_f:
            metadata_f.write(metadata)
        metadata_f.close()

        #pin to pinata
        hash_pinata = _pin_to_pinata(packagePath + "/data/metadata_" + name + "_testnet.json")
        rospy.loginfo(hash_pinata)
    else:
        with open(packagePath + "/data/metadata_" + name + "_mainnet.json", 'w') as metadata_f:
            metadata_f.write(metadata)
        metadata_f.close()

        #pin to pinata
        hash_pinata = _pin_to_pinata(packagePath + "/data/metadata_" + name + "_mainnet.json")
        rospy.loginfo(hash_pinata)

    #buildup txn
    gas_estimate = contract.functions.mintWithURI(to_who, hash_pinata).estimateGas()
    rospy.loginfo("gas_estimate " + str(gas_estimate))
    if not testnet:
        w3.eth.setGasPriceStrategy(medium_gas_price_strategy)
        gas_price = w3.eth.generateGasPrice()
        rospy.loginfo("gas_price " + str(gas_price))
        transaction_price = gas_estimate*gas_price
        rospy.loginfo("transaction_price " + str(Web3.fromWei(transaction_price, 'ether')))

    transaction = contract.functions.mintWithURI(to_who, hash_pinata).buildTransaction()
    rospy.loginfo('Transaction: ' + str(transaction))
    transaction.update({ 'gas' : gas_estimate })
    transaction.update({ 'nonce' : w3.eth.get_transaction_count(minter) })
    if not testnet:
        transaction.update({ 'gasPrice' : gas_price })
    signed_tx = w3.eth.account.sign_transaction(transaction, minter_seed)

    #send_txn
    txn_hash = w3.eth.send_raw_transaction(signed_tx.rawTransaction)
    txn_receipt = w3.eth.waitForTransactionReceipt(txn_hash)
    rospy.loginfo("txn_receipt: " + str(txn_receipt))

def listener(packagePath):
    rospy.init_node('NFT_minter', anonymous=False)
    callback_lambda = lambda x: callback(x,packagePath)
    rospy.Subscriber("NFT_data", String, callback_lambda)
    rospy.spin()


if __name__ == '__main__':

    #locate itself
    rospack = rospkg.RosPack()
    packagePath = rospack.get_path('nft_minter') + "/"

    #get config parameters
    config = read_configuration(packagePath)
    testnet = config['general']['testnet']
    pinata_api = config['parameters']['pinata_api']
    pinata_secret_api = config['parameters']['pinata_secret_api']
    to_who = config['parameters']['to_who']
    minter = config['parameters']['minter']
    minter_seed = config['parameters']['minter_seed']
    contract_filename = config['parameters']['contract_filename']
    if testnet:
        provider = config['parameters']['provider_testnet']
        contract_address = config['parameters']['contract_address_testnet']
    else:
        provider = config['parameters']['provider']
        contract_address = config['parameters']['contract_address']


    #connect to node
    w3 = Web3(Web3.WebsocketProvider(provider))
    if testnet:
        logging.warning('Is connected to testnet: ' + str(w3.isConnected()))
    else:
        logging.warning('Is connected to mainnet: ' + str(w3.isConnected()))


    #initiate contract connection
    contract_file_path = packagePath + "data/" + contract_filename
    with open(contract_file_path, 'r') as f:
        contract_file = json.loads(f.read())

    contract = w3.eth.contract(address=contract_address, abi=contract_file["abi"])
    w3.eth.default_account = minter
    #ros node
    listener(packagePath)
