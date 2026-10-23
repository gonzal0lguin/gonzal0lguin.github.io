// Talking to the editor API (editor/server/api.mjs). Development only.

export const API = "/__editor/api";

async function request<T>(method: string, path: string, init: RequestInit = {}): Promise<T> {
  const response = await fetch(`${API}${path}`, { method, ...init });
  const text = await response.text();
  let data: unknown = null;
  try {
    data = text ? JSON.parse(text) : null;
  } catch {
    throw new Error(`${method} ${path}: ${text.slice(0, 300)}`);
  }
  if (!response.ok) {
    const error = new Error((data as { error?: string })?.error ?? `${method} ${path} failed`);
    (error as Error & { status?: number }).status = response.status;
    throw error;
  }
  return data as T;
}

export const get = <T>(path: string, params: Record<string, string> = {}) => {
  const query = new URLSearchParams(params).toString();
  return request<T>("GET", query ? `${path}?${query}` : path);
};

export const post = <T>(path: string, body?: unknown) =>
  request<T>("POST", path, {
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body ?? {}),
  });

/** Upload one file; the bytes are the request body, so there is no multipart parsing anywhere. */
export async function uploadFile(file: File, options: { kind?: "photo" | "header" | "gpx"; folder?: string } = {}) {
  const params = new URLSearchParams({ kind: options.kind ?? "photo", name: file.name });
  if (options.folder) params.set("folder", options.folder);
  return request<UploadResult>("POST", `/media/upload?${params}`, {
    headers: { "content-type": file.type || "application/octet-stream" },
    body: file,
  });
}

/** Overwrite one file of public/ in place, keeping its path (the CV PDF, the profile photo...). */
export async function replaceFile(path: string, file: File) {
  return request<UploadResult>("POST", `/media/replace?path=${encodeURIComponent(path)}`, {
    headers: { "content-type": file.type || "application/octet-stream" },
    body: file,
  });
}

/** Run one of the allowed npm scripts, reporting its output as it arrives. */
export async function runTask(task: TaskName, files: string[], onOutput: (chunk: string) => void) {
  const response = await fetch(`${API}/task`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ task, files }),
  });
  const reader = response.body?.getReader();
  if (!reader) throw new Error("No output from the task");
  const decoder = new TextDecoder();
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    onOutput(decoder.decode(value, { stream: true }));
  }
}

export type TaskName = "photos" | "translate" | "stamp" | "build";

export interface UploadResult {
  path: string;
  relPath: string | null;
  publicPath: string;
  size: number;
}

export interface TranslationInfo {
  path: string;
  hash: string | null;
  state: "fresh" | "stale" | "manual";
}

export interface EntrySummary {
  collection: string;
  key: string;
  path: string;
  mtime: number;
  title: string;
  description: string;
  pubDate: string | null;
  badge: string | null;
  category: string | null;
  tags: string[];
  heroImage: string | null;
  route: string[];
  translation: TranslationInfo | null;
}

export interface CollectionInfo {
  id: string;
  label: string;
  urlBase: string | null;
  mediaFolder: boolean;
  entries: EntrySummary[];
}

export interface GpxInfo {
  path: string;
  publicPath: string;
  name: string;
  size: number;
  mtime: number;
  usedBy: string[];
}

export interface GitFile {
  path: string;
  staged: string | null;
  unstaged: string | null;
  untracked: boolean;
}

export interface GitStatus {
  branch?: string;
  files: GitFile[];
  log: { hash: string; when: string; subject: string }[];
  error?: string;
}

export interface EditorState {
  root: string;
  mediaBaseUrl: string | null;
  localDir: string;
  collections: CollectionInfo[];
  gpx: GpxInfo[];
  git: GitStatus;
}

export interface MediaFile {
  path: string;
  name: string;
  relPath: string | null;
  folder: string | null;
  size: number;
  mtime: number;
  usedBy: string[];
  missing?: boolean;
}

export interface MediaLibrary {
  mediaBaseUrl: string | null;
  postsDir: string;
  headersDir: string;
  folders: string[];
  posts: MediaFile[];
  headers: MediaFile[];
}

export interface HealthIssue {
  level: "error" | "warn" | "info";
  kind: string;
  message: string;
  path: string;
  collection?: string;
  key?: string;
  title?: string;
}

export interface HealthReport {
  issues: HealthIssue[];
  unusedPhotos: string[];
  counts: { entries: number; errors: number; warnings: number };
}

export interface FileContent {
  path: string;
  text: string;
  mtime: number;
}
