// Which game this page is (DESIGN.md §18): the one served by the game's server (`duels`), with
// every firmware behind it, or the web version, a static page whose simulator runs in a worker
// (sim/) and which has nothing to ask but files. The build says which (scripts/build.ts
// defines DUELS_WEB); under `bun test` it is not defined, and the page is the server's.

declare const DUELS_WEB: boolean | undefined;

export const WEB: boolean = typeof DUELS_WEB !== "undefined" && DUELS_WEB === true;

/** A file of the page's own, by its path from the page: the web version is served from a
 * folder (GitHub Pages: /<repository>/), so nothing of its own is asked for from the root. */
export const asset = (path: string): string => new URL(path, document.baseURI).href;
