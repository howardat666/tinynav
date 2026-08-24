// ignore: avoid_web_libraries_in_flutter
import 'dart:html' as html;

/// The backend already sends Content-Disposition: attachment, so a same-origin
/// anchor click is enough -- and it avoids the popup blocker that window.open hits.
void downloadFile(String url, String filename) {
  final a = html.AnchorElement(href: url)..setAttribute('download', filename);
  html.document.body?.append(a);
  a.click();
  a.remove();
}
