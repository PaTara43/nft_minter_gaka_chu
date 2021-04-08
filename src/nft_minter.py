#!/usr/bin/python3

import json
import logging
import os
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

def rename(filepath, name, file_format, net):
    res = filepath[0:filepath.rfind('/')+1] + filepath[filepath.rfind('.'):]
    res = res[0:res.rfind('/')+1] + name + '_' + file_format + '_' + net + res[res.rfind('/')+1:]
    return res

def callback(data, packagePath):

    #read topic
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    name = data.data

    #find files in their known locations
    rospy.loginfo('Finding media file paths')
    video_filepath = config['filepaths']['video']
    picture_filepath = config['filepaths']['picture']
    rosbag_filepath = config['filepaths']['rosbag']

    #rename files
    rospy.loginfo('Creating new names')
    if testnet:
        video_filepath_renamed = rename(video_filepath, name, 'video', 'testnet')
        picture_filepath_renamed = rename(picture_filepath, name, 'picture', 'testnet')
        rosbag_filepath_renamed = rename(rosbag_filepath, name, 'rosbag', 'testnet')
    else:
        video_filepath_renamed = rename(video_filepath, name, 'video', 'mainnet')
        picture_filepath_renamed = rename(picture_filepath, name, 'picture', 'mainnet')
        rosbag_filepath_renamed = rename(rosbag_filepath, name, 'rosbag', 'mainnet')

    rospy.loginfo('Renaming files')
    try:
        os.rename(video_filepath, video_filepath_renamed)
        os.rename(picture_filepath, picture_filepath_renamed)
        os.rename(rosbag_filepath, rosbag_filepath_renamed)
    except FileNotFoundError as e:
        rospy.logwarn('Perhaps, files have already been renamed. Trying to find them...')
        if (not os.path.exists(video_filepath_renamed)) or (not os.path.exists(picture_filepath_renamed)) or (not os.path.exists(rosbag_filepath_renamed)):
            rospy.logerr('No media files found. Breaking')
            return False
        else:
            rospy.loginfo('Files have already been renamed. Going on')
            pass

    #pin them to pinata:
    rospy.loginfo('Upolading media to pinata')
    hash_video = _pin_to_pinata(video_filepath_renamed)
    hash_picture = _pin_to_pinata(picture_filepath_renamed)
    hash_rosbag = _pin_to_pinata(rosbag_filepath_renamed)

    #json metadata
    rospy.loginfo('Creating metadata file')
    description = 'This is a sample description for a NFT minted by robot. Log available at ipfs://' + hash_rosbag + '.'

    metadata_d = {
      "description": description,
      "external_url": "https://dapp.robonomics.network",
      "image": "ipfs://" + hash_picture,
      "name": name.replace("_", " ") + ". Artwork by robot-artist Gaka-Сhu",
      "animation_url": "ipfs://" + hash_video,
    }
    metadata = json.dumps(metadata_d, indent=2)

    rospy.loginfo('Uploading metadata to pinata')
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
    rospy.loginfo('Building up transaction')
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
    rospy.loginfo('Sending transaction')
    txn_hash = w3.eth.send_raw_transaction(signed_tx.rawTransaction)
    txn_receipt = w3.eth.waitForTransactionReceipt(txn_hash)
    rospy.loginfo("txn_receipt: " + str(txn_receipt))

def listener(packagePath):
    rospy.loginfo('Initiating node')
    rospy.init_node('NFT_minter', anonymous=False)
    callback_lambda = lambda x: callback(x,packagePath)
    rospy.Subscriber("NFT_data", String, callback_lambda)
    rospy.loginfo('Listening to NFT_minter')
    rospy.spin()


if __name__ == '__main__':

    #locate itself
    rospack = rospkg.RosPack()
    packagePath = rospack.get_path('nft_minter') + "/"

    #get config parameters
    rospy.loginfo('Searching for config')
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
    rospy.loginfo('Configuration set')


    #connect to node
    w3 = Web3(Web3.WebsocketProvider(provider))
    if testnet:
        rospy.loginfo('Is connected to testnet: ' + str(w3.isConnected()))
    else:
        rospy.loginfo('Is connected to mainnet: ' + str(w3.isConnected()))


    #initiate contract connection
    rospy.loginfo('Initiating contract connection')
    contract_file_path = packagePath + "data/" + contract_filename
    with open(contract_file_path, 'r') as f:
        contract_file = json.loads(f.read())

    contract = w3.eth.contract(address=contract_address, abi=contract_file["abi"])
    w3.eth.default_account = minter
    rospy.loginfo('Connected to contract')
    #ros node
    listener(packagePath)