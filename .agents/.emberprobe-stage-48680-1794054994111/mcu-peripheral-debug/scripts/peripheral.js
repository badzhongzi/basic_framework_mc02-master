"use strict";

const { call, writeDiagnostic } = require("../../_emberprobe/agent-client");

function args(argv) {
    const out = {};
    for (let index = 0; index < argv.length; index += 1) {
        const key = argv[index];
        if (key === "--list") out.list = true;
        else if (["--workspace", "--query", "--peripheral", "--read", "--set", "--confirm"].includes(key)) {
            if (!argv[index + 1]) throw new Error(`Missing value for ${key}`);
            out[key.slice(2)] = argv[++index];
        } else throw new Error(`Unknown argument: ${key}`);
    }
    if (out.confirm !== undefined && out.set === undefined) throw new Error("--confirm can only be used with --set");
    return out;
}

function parseSet(value) {
    const writes = [];
    for (const part of String(value || "")
        .split(",")
        .map((item) => item.trim())
        .filter(Boolean)) {
        const equals = part.indexOf("=");
        if (equals < 1 || equals === part.length - 1)
            throw new Error(`Invalid peripheral assignment (expected TARGET=value): ${part}`);
        writes.push({ target: part.slice(0, equals).trim(), value: part.slice(equals + 1).trim() });
    }
    if (!writes.length) throw new Error("--set requires at least one TARGET=value pair");
    return writes;
}

async function main() {
    const opt = args(process.argv.slice(2));
    const workspace = opt.workspace || process.cwd();
    const actions = [!!opt.list, !!opt.read, !!opt.set].filter(Boolean).length;
    if (actions > 1) throw new Error("Choose exactly one of --list, --read, or --set");
    let method = "peripherals.list";
    /** @type {Record<string, any>} */
    let params = { query: opt.query, peripheral: opt.peripheral };
    if (opt.read) {
        method = "peripherals.read";
        params = {
            targets: opt.read
                .split(",")
                .map((target) => target.trim())
                .filter(Boolean)
        };
    } else if (opt.set) {
        method = "peripherals.write";
        params = { writes: parseSet(opt.set), confirmationId: opt.confirm };
    }
    const result = await call(workspace, method, params);
    process.stdout.write(`${JSON.stringify(result)}\n`);
}

if (require.main === module)
    main().catch((error) => {
        const operation = process.argv.includes("--set")
            ? "peripherals.write"
            : process.argv.includes("--read")
              ? "peripherals.read"
              : "peripherals.list";
        writeDiagnostic(error, { operation });
        process.exitCode = 1;
    });

module.exports = { args, parseSet };
