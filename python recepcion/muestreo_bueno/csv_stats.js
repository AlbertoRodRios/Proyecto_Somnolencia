//@ts-check
/**
 * Small script .csv statistics extractor
 * Just change the readFileSync("pathhere") to .csv file to read from
 */

import fs from "fs";

//! CHANGE FILE PATH TO ANALIZE CSV STATISTICS
/** @type {string[]} */
const lines = fs.readFileSync("./featuresSomnolencia_full_2900.csv").toString().split("\n").slice(1);

/** @type {Map<string, {awakeCount:number, sleepCount: number}>} */
const usersMap = new Map();
lines.forEach(current => {
  const byColumns = current.split(",");
  const user = byColumns[0];
  if (!usersMap.has(user)) {
    usersMap.set(user, {
      awakeCount: 0,
      sleepCount: 0,
    });
  }
  const currentLineTrimed = current.trim();
  const lastChar = currentLineTrimed.at(-1);
  const awake = lastChar === "0" ? false : lastChar === "1" ? true : null;
  if (awake === null) {
    throw "WTF THIS SHOULDN'T HAPPEN";
  }
  const actualUserRegistry = usersMap.get(user);
  if (actualUserRegistry) {
    if (awake) {
      actualUserRegistry.awakeCount++;
    } else {
      actualUserRegistry.sleepCount++;
    }
  }
}, 0);

console.log(usersMap);
