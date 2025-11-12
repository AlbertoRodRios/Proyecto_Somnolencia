//@ts-check
/**
 * Small script .csv statistics extractor
 * Just change the readFileSync("pathhere") to .csv file to read from
 */
import path from "path";
import fs from "fs";
import { createGunzip } from "zlib";

const CSV_FILENAME = "./featuresSomnolencia_full_2900.csv";

const lines = fs.readFileSync(CSV_FILENAME).toString().split("\n").slice(1);

/** @type {string[]} */
const goodLines = lines.filter(currentLine => {
  const loweredCase = currentLine.toLowerCase();
  if (loweredCase.includes("ovf")) {
    return false;
  }
  return true;
});

if (goodLines.length > 0) {
  const newFilePathName = CSV_FILENAME.replace(path.extname(CSV_FILENAME), "") + "_NO_OSF.csv";

  if (fs.existsSync(newFilePathName)) {
    fs.rmSync(newFilePathName);
  }
  fs.writeFileSync(newFilePathName, goodLines.join("\n"));
  console.log(newFilePathName);
  console.log("CREATED STUFF");
}

console.log(goodLines.length);
