const logger = require('./logger');
const mqtt = require('mqtt');
const EventEmitter = require('events');
const myEmitter = new EventEmitter();

// Map of robots and disconnected robots
const robots = new Map();
const disconnected = new Map();

// Used to dismiss robots that process disks at a fast rate (3 per 10 seconds is unrealistic)
const robotTinyInterval = new Map();

/** 
 * Generate a number of a disk (something like an id of the disk)
 */ 
function* generateNumbers(n) {
  for (let i = 1; i <= n; i++) {
    yield i;
  }
}
const diskNumberGenerator = generateNumbers(1000);

/**
 * Function that checks if by taking the next disk the robot will have 
 * an approximately equal number of processed disks as the others.
 * The difference between any 2 robots'processed disks should be < 3.
 * However, if some robot has less than 2 processed disks than another robot
 * (if this robot has been disconnected, and the other has processed disks 
 * while it was gone; normally does not happen) it is able to take the next disk.
 */ 
function checkRobot(robotName) {
  for (const [key, value] of robots) {
    if (value - robots.get(robotName) > 2) {
      return 1;
    }
    if (Math.abs(value - robots.get(robotName) - 1) > 2) {
      return 0;
    }
  }
  return 1;
}

/**
 * Function that publishes in each robot's 'allowed' topic whether it 
 * can process the next disk.
 */
function informRobots() {
  console.log('-------------------------------');
  for (const [robot, _] of robots) {
    const resp = checkRobot(robot);
    client.publish(`robots/allowed/${robot}`, resp.toString(), { qos: 1 });
    // just for illustration purposes
    if (resp === 1) {
      console.log(`${robot} can process the next disk.`);
    } else {
      console.log(`${robot} can NOT process the next disk.`);
    }
  }
  console.log('-------------------------------');
  console.log();
}

/**
 * Keeps track of the number of processed disks by each robot in the last 10 seconds.
 * If a robot sends 1 in its progress topic at a really high rate, it is broken.
 */
async function tinyIntervalProcessedDisk(nameRobot) {
  robotTinyInterval.set(nameRobot, (robotTinyInterval.get(nameRobot) || 0) + 1);
  if (robotTinyInterval.get(nameRobot) >= 3) {
    client.publish(`robots/errors/${nameRobot}`, 'processing at high rate', { qos : 1 });
    logger.logTime(`${nameRobot} is probably broken. :(`);
    //client.publish(`robots/errors/${nameRobot}`, 'processing at high rate', { qos : 1 });
    robots.delete(nameRobot);
    informRobots();
  }
  await new Promise((resolve) => setTimeout(resolve, 10000));
  robotTinyInterval.set(nameRobot, (robotTinyInterval.get(nameRobot) || 0) - 1);
}

/**
 * When a disk is processed, it is added to the map and the robots are informed.
 */
async function handleProcessedDisk(nameRobot) {
  robots.set(nameRobot, robots.get(nameRobot) + 1);
  tinyIntervalProcessedDisk(nameRobot);
  const diskID = diskNumberGenerator.next().value;
  logger.logTime(`${nameRobot} just processed a disk ${diskID}`);
  console.log(robots);
  informRobots();

}

// Connect to the MQTT broker 
const mqttBroker = 'mqtt://localhost';
const client = mqtt.connect(mqttBroker, { clientId: 'superclient' });

/**
 * On connecting, the client subscribes to all progress topics and the 
 * disconnected topic.
 */
client.on('connect', () => {
  client.subscribe('robots/progress/#', { qos: 1 });
  client.subscribe('disconnected', { qos: 1 });
  // This is if some robot wants to signal that it is broken
  client.subscribe('robots/errors/#', { qos: 1 });
});

/**
 * Same as previous version, but when the robot processes a disk, event 'processedDisk' is emitted
 */
client.on('message', (topic, message) => {
  const topicParts = topic.split('/');
  const nameRobot = topicParts[topicParts.length - 1];

  // LWT messages, handles disconnected robots
  if (topic === 'disconnected') {
    const disconnectedRobot = message.toString();
    if (robots.has(disconnectedRobot)) {
      disconnected.set(disconnectedRobot, robots.get(disconnectedRobot));
      robots.delete(disconnectedRobot);
      logger.logTime(`${disconnectedRobot} left`);
      informRobots();
      console.log();
      
    }
  }

  // Robot processes a disk in the progress topic
  else if (topicParts[1] === "progress" && parseInt(message, 10) === 1 && robots.has(nameRobot)) {
    handleProcessedDisk(nameRobot);
  }
  
  // robot reconnects : message 2 in the progress toppic
  else if (topicParts[1] === "progress" && parseInt(message, 10) === 2) {
    // move from disconnected to robots
    robots.set(nameRobot, disconnected.get(nameRobot));
    disconnected.delete(nameRobot);
    logger.logTime(`${nameRobot} reconnected`);
    informRobots();

  }
  
  // Robots publish 0 in the progress topic, when they connect -> new element in the map
  else if (topicParts[1] === "progress" && parseInt(message, 10) === 0) {
    robots.set(nameRobot, 0);
    robotTinyInterval.set(nameRobot, 0);
    logger.logTime(`${nameRobot} joined`);
    
    console.log(robots);
    informRobots();
  }
});


// Handle uncaught exceptions and exit gracefully
process.on('uncaughtException', (err) => {
  console.error('Uncaught Exception:');
  console.error(err);
  robots.forEach((_, robot) => {
    client.publish(`robots/errors/${robot}`, 'Uncaught Exception', { qos: 1 });
  });
  process.exit(1);
});

// Handle unhandled promise rejections and exit gracefully
process.on('unhandledRejection', (reason, promise) => {
  console.error('Unhandled Promise Rejection:');
  console.error(reason);
  robots.forEach((_, robot) => {
    client.publish(`robots/errors/${robot}`, 'Unhandled Promise Rejection', { qos: 1 });
  });
  process.exit(1);
});
