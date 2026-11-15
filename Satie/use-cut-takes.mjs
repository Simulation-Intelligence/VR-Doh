// Copies the cut takes in Satie/takes/ over the downloaded ones in .satie/takes/.
//
// Satie's generated impacts held several hits each (a "single" mallet strike had 2-3 thuds), so
// every impact slot is cut to one hit, and the dough-release takes to about 0.8 s; Satie/take-edits.json
// lists each cut. The satie-unity line compiles whatever is in .satie/takes/ and downloads only what is
// missing there, so on a fresh checkout it would compile the uncut downloads. Run this first:
//
//   node Satie/use-cut-takes.mjs
//   npx -y https://www.satie.world/unity/0.1.0/satie-unity.tgz      (the line satie_export gave, without --from)
//
// Run both from the Unity project root (the folder that holds Assets).
import { copyFileSync, existsSync, mkdirSync, readFileSync } from "node:fs";
import { join } from "node:path";

const root = process.cwd();
if (!existsSync(join(root, "Assets"))) {
  console.error("Run this from the Unity project root (the folder that holds Assets).");
  process.exit(1);
}
const edits = JSON.parse(readFileSync(join(root, "Satie", "take-edits.json"), "utf8"));
const target = join(root, ".satie", "takes");
mkdirSync(target, { recursive: true });
let copied = 0;
for (const [clip, cut] of Object.entries(edits.cuts)) {
  const from = join(root, "Satie", "takes", cut.file);
  if (!existsSync(from)) {
    console.error(`missing ${from} (${clip})`);
    process.exitCode = 1;
    continue;
  }
  copyFileSync(from, join(target, cut.file));
  copied++;
}
console.log(`Copied ${copied} cut takes into .satie/takes/. Now run the satie-unity line to compile them.`);
