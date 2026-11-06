# 展示会紹介ポスター

- [編集用PowerPoint](robot_arm_exhibition_A4.pptx)
- [印刷用PDF](robot_arm_exhibition_A4.pdf)
- [プレビュー画像](robot_arm_exhibition_A4_preview.png)
- [展示説明者向け 技術解説・想定問答集](technical_guide.md)：短い説明例，工夫点，ArUco・時間窓・衝突判定・モーター制御・到達誤差への対応を解説．

A4縦・1ページ。対象は子どもとロボットに詳しくない来場者。大見出し、2枚の目印の説明図、動作の3ステップ、カメラ→パソコン→ロボットの簡易アーキテクチャ図で構成。

PowerPointの文章・図形・イラストは編集可能。推奨フォントはNoto Sans CJK JP。未導入のPCではフォントが置き換わる場合があるので、印刷にはPDFを使用するとレイアウトを保てる。図はSO-101のしくみを説明する模式図で、実機の正確な形状図ではない。AIによる物体認識や生成AIによる制御など、実装していない機能は説明に含めていない。

印刷はA4縦・1枚。余白の都合でプリンターが縮小する場合は見切れがないことを確認する。マーカーイラストは説明用で、校正や位置推定用の実寸印刷物ではない。

再生成スクリプトは `scripts/create_exhibition_poster.py`。Python依存はpython-pptx、reportlab、Pillow、lxml。日本語PDFの埋め込みには `/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf` を使用。生成時は一時フォルダー `/tmp/so101_poster_deps` に依存を取得し、システム環境・ロボット制御依存は変更していない。PDFは日本語フォントを埋め込んだベクトル形式。PowerPointにも図形・文字を個別に格納している。

検証: A4寸法と1スライドであること、PowerPointパッケージの全XMLを読み込めること、196個の編集可能な要素を確認。PDFを画像化して目視でレイアウト・日本語／英数字の欠けを確認。PowerPointアプリ上での最終レンダリングは未確認。
