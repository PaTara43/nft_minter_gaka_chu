const opensea = require("opensea-js");
const HDWalletProvider = require("@truffle/hdwallet-provider");
const OpenSeaPort = opensea.OpenSeaPort;

const Network = opensea.Network;

const NODE_API_KEY = process.argv[2];
const SEED = [process.argv[3]];
const OWNER_ADDRESS = process.argv[4];
const NETWORK = process.argv[5];
const NFT_CONTRACT_ADDRESS = process.argv[6];

const DURATION = parseInt(process.argv[7], 10);
const START_AMOUNT = parseFloat(process.argv[8]);

//
// console.log(NODE_API_KEY)
// console.log(SEED)
// console.log(OWNER_ADDRESS)
// console.log(NETWORK)
// console.log(NFT_CONTRACT_ADDRESS)
//
// console.log(TOKEN_ID)
// console.log(DURATION)
// console.log(START_AMOUNT)
// console.log(END_AMOUNT)
// console.log(RESERVE_PRICE)



const provider = new HDWalletProvider({
  privateKeys: SEED,
  providerOrUrl:
        NETWORK === "mainnet"
          ? "https://mainnet.infura.io/v3/" + NODE_API_KEY
          : "https://rinkeby.infura.io/v3/" + NODE_API_KEY,
});

const seaport = new OpenSeaPort(provider, {
  networkName:
        NETWORK === "mainnet"
          ? Network.Main
          : Network.Rinkeby,
});


async function main() {

  /////////////////////////////////
  // Testing connection and etching total number of items
  /////////////////////////////////

  console.log("Fetching total number of items")
  const asset = await seaport.api.getAsset({
    tokenAddress: NFT_CONTRACT_ADDRESS, // string
  })
  console.log(asset.collection.stats.count-1)
  const TOKEN_ID = asset.collection.stats.count-1 // Latest

  /////////////////
  //English auction
  /////////////////

  console.log("English auctioning an item in WETH...");
  const expirationTime = Math.round(Date.now() / 1000 + DURATION);
  console.log(Math.round(Date.now() / 1000))
  console.log(expirationTime)
  const wethAddress =
    NETWORK === "mainnet" || NETWORK === "live"
      ? "0xc02aaa39b223fe8d0a0e5c4f27ead9083c756cc2"
      : "0xc778417e063141139fce010982780140aa0cd5ab";

  try{
    const englishAuctionSellOrder = await seaport.createSellOrder({
      asset: {
        tokenId: TOKEN_ID,
        tokenAddress: NFT_CONTRACT_ADDRESS,
      },
      accountAddress: OWNER_ADDRESS,
      startAmount: START_AMOUNT,
      expirationTime: expirationTime,
      waitForHighestBid: true,
      paymentTokenAddress: wethAddress,

    });
    console.log(
      `Successfully created an English auction sell order! ${englishAuctionSellOrder.asset.openseaLink}\n`
    );
  } catch (err) {
    console.log(err)
  }

  process.exit()
}
main();
