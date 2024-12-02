# RP_Balance

## 前書き

<div align="center">
<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/%E5%A4%87%E8%B5%9B%E8%AE%B0%E5%BD%95/2024_1.jpg" width="710px">
</div>
<br>
<p align="center">本プロジェクトは、2023～2024年に深セン大学RobotPilots研究室で完成しました。新しい研究室メンバーの学習参考用に設計されており、2024年末まで継続的に更新される予定です。エラーや間違いがあれば、ぜひご指摘ください。</p>

## モデリングとコントローラ設計

| ファイル名 | 機能 | ファイルリンク |
|---|---|---|
| HEU_Model | ハルビン工科大学モデルに基づく数学的モデリングとコントローラ設計 | [HEU](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/HEU_Model) |
| SJTU_Model | 全身モデルとコントローラ設計、五連結リンク解法、ゲインマトリックスフィッティング、ABマトリックスフィッティング | [SJTU](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/SJTU_Model) |
| Webots_Data_Estimate | シミュレーションデータの分析 | [DATA](https://github.com/WilliamGwok/RP_Balance/tree/main/MatlabWorks/Webots_Data_Estimate) |

### コアコード
#### バランスシャーシモデリング
動力学および運動学方程式を構築し、簡略化してシステムAおよびBマトリックスを計算します。

| コード | 機能 | ファイルリンク |
|---|---|---|
| Bot_Dynamics_Part1 | 方程式を作成して簡略化し、AおよびBマトリックスを取得（P1～P3） | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/Model/Bot_Dynamics_Part1.m) |

#### 五連結リンク解法とVMC制御
角度関係および仮想力の解法。

| コード | 機能 | ファイルリンク |
|---|---|---|
| Centroid | リンクの重心座標と慣性モーメントを解く | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/Model/Centroid.m) |
| Five_Link_Base | 角度関係の解法と仮想力の計算 | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/Model/Five_Link_Base.m) |

#### LQRコントローラとKマトリックスフィッティング
この車輪型シャーシは五連結リンク構造を採用しており、脚の長さが可変です。そのため、脚の長さの変化に伴い、シャーシモデルが変化します。この変化に応じて、Kマトリックスのフィッティング係数を計算する必要があります。

| コード | 機能 | ファイルリンク |
|---|---|---|
| Master | フィッティングのメイン関数 | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/K_L_Fitting/Master.m) |
| Func_Cal_K | Kマトリックスの計算 | [.m](https://github.com/WilliamGwok/RP_Balance/blob/main/MatlabWorks/SJTU_Model/K_L_Fitting/Func_Cal_K.m) |

#### システムA、BマトリックスフィッティングとMPC風制御
ロボットの駆動輪が滑らかな路面や起伏のある路面で突然の抵抗に遭遇した場合の不安定性を解決するために、MPC（モデル予測制御）風のアプローチを採用しています。このアプローチでは、ロボットの現在の状態と入力に基づいて次の状態を予測し、予測された状態と実際の状態の差に基づいて駆動輪のトルクを補正します。

A、Bマトリックスもフィッティングを必要とするため、この方法を採用しています。

## シミュレーション

<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/Webots/PixPin_2024-10-22_23-50-17.png" width="410px">

[メインシミュレーションコード](https://github.com/WilliamGwok/RP_Balance/tree/main/Webots/LinkLinkGo/controllers/Eva_Test_02)

| ファイル名 | 機能 |
|---|---|
| Config | シミュレーション環境設定、ロボットの機械的パラメータ設定など |
| Device | モーター、エンコーダー、IMUなどのデバイス |
| Model | 直脚モデル、仮想脚力計算など、ロボットモデルの階層 |
| Algorithm | PIDアルゴリズム、五連結リンク解法 |
| Robot | ロボットのトップレベル制御コード |

## テスト動画
[その他のテスト動画](https://github.com/WilliamGwok/RP_Balance/tree/main/Test_Video)

#### シミュレーションテスト
https://github.com/user-attachments/assets/7b33e5dd-0533-4bbb-9497-3062cb68b34d

#### 実車テスト
https://github.com/user-attachments/assets/42d3e697-0503-4a66-8977-5c767b99fcbf

## 実車デバッグの詳細

| ファイル名 | ファイルリンク |
|---|---|
| LQRコントローラのパラメータ調整のコツ | [NOTE_1](https://github.com/WilliamGwok/RP_Balance/blob/main/%E5%AE%9E%E8%BD%A6%E8%B0%83%E8%AF%95%E7%BB%86%E8%8A%82/%E5%85%B3%E4%BA%8ELQR%E6%8E%A7%E5%88%B6%E5%99%A8%E7%9A%84%E5%8F%82%E6%95%B0%E6%95%B4%E5%AE%9A%E6%8A%80%E5%B7%A7.md) |
| 駆動輪モーターがリンク回転中に相対的に移動した結果生じた移動計算エラーの分析 | [NOTE_2](https://github.com/WilliamGwok/RP_Balance/blob/main/%E5%AE%9E%E8%BD%A6%E8%B0%83%E8%AF%95%E7%BB%86%E8%8A%82/%E5%85%B3%E4%BA%8E%E9%A9%B1%E5%8A%A8%E8%BD%A6%E7%94%B5%E6%9C%BA%E5%9C%A8%E8%BF%9E%E6%9D%86%E8%BD%AC%E5%8A%A8%E6%97%B6%E5%AE%9A%E8%BD%AC%E7%9B%B8%E5%AF%B9%E7%A7%BB%E5%8A%A8%E5%AF%BC%E8%87%B4%E4%BD%8D%E7%A7%BB%E8%AE%A1%E7%AE%97%E9%94%99%E8%AF%AF%E7%9A%84%E9%97%AE%E9%A2%98.md) |
| NULL | To be updated |

## デバッグの苦労

#### 一貫したモータープロテクションの問題
2024年、研究室のハードウェアが制約されていたため、テスト中にモーターが頻繁に焼損し、時には発火することさえありました。経験豊富なメンバーの支援により、より安定した保護システムが最終的に実装されました。また、競技中には絶縁損傷によるハードウェア故障も発生しました。

<div align="center">
<img src="https://github.com/WilliamGwok/RP_Balance/blob/main/Figures/%E5%A4%87%E8%B5%9B%E8%AE%B0
