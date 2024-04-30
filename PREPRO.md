# 点群プリプロセッサー(preproX.py)

## 仕様
<table>
<tr><th>モード<th>名称<th>機能</tr>
<tr><td>0<td>何もしない<td>通常のVTと等価機能</tr>
<tr><td>1<td>ワークピック<td><ul><li>ピック対象の選択<li>対象物の大まかな位置計測</ul></tr>
<tr><td>2<td>バケット位置出し<td><ul><li>バケットの位置出しピック対象の選択<li>荷量の計測</ul></tr>
</table>

## コンポーネント図

## 戻り値(OK時)
### モード0
<table>
<tr><th>Request<th>Response<th>Value</tr>
<tr><td rowspan="4">X1<br>Capture<td>戻値1(候補)<td>不定</tr>
<tr><td>戻値2(柱距離)<td>不定</tr>
<tr><td>戻値3<td>不定</tr>
<tr><td>戻値4<td>不定</tr>
<tr><td rowspan="2">X2<br>Solve<td>戻値1〜6<td>Base変換</tr>
<tr><td>一致度<td>マスタ一致度</tr>
</table>

### モード1
<table>
<tr><th>Request<th>Response<th>Value</tr>
<tr><td rowspan="4">X1<br>Capture<td>戻値1(候補)<td>候補数</tr>
<tr><td>戻値2(柱距離)<td>候補位置の柱からの距離(mm)。負値は反対側(ロボットから見て右)の柱基準。</tr>
<tr><td>戻値3<td>未定義(常時0)</tr>
<tr><td>戻値4<td>未定義(常時0)</tr>
<tr><td rowspan="2">X2<br>Solve<td>戻値1〜6<td>Base変換</tr>
<tr><td>一致度<td>マスタ一致度</tr>
</table>

### モード2
<table>
<tr><th>Request<th>Response<th>Value</tr>
<tr><td rowspan="4">X1<br>Capture<td>戻値1(候補)<td>0:残量計測失敗<br>1:残量計測成功</tr>
<tr><td>戻値2(柱距離)<td>残量計測成功時、バケット基準位置から積荷までの距離(mm)</tr>
<tr><td>戻値3<td>未定義(常時0)</tr>
<tr><td>戻値4<td>未定義(常時0)</tr>
<tr><td rowspan="2">X2<br>Solve<td>戻値1〜6<td>バケット座標(Base座標系)</tr>
<tr><td>一致度<td>スライスされた点数/探索点数</tr>
</table>
