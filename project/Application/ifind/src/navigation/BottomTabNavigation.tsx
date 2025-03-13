import { BottomTabBar, BottomTabBarProps, createBottomTabNavigator } from '@react-navigation/bottom-tabs';
import React from 'react';
import { Image, SafeAreaView, StyleSheet } from 'react-native';
import { AlertPage } from '../screens/AlertPage';
import { MainPage } from '../screens/MainPage';
import { MyPage } from '../screens/Mypage';
import { NodePage } from '../screens/NodePage';
import { SettingPage } from '../screens/Settingpage';
import theme from '../styles/theme';
const Tab = createBottomTabNavigator();

const colorchange = theme.navicolor;

const BottomTabNavigation = () => {
    return (
            <Tab.Navigator
                tabBar={(props: BottomTabBarProps) => (
                    <SafeAreaView style={styles.container}>
                        <BottomTabBar {...props} />
                    </SafeAreaView>
                )}
                screenOptions={{
                    tabBarStyle: {
                        height: 60,
                        paddingBottom: 10,
                        paddingTop: 10,
                    },
                    tabBarShowLabel: false,
                }}
            >
                

                {/* 마이페이지 */}
                <Tab.Screen name="My" component={MyPage}
                options={{
                    tabBarIcon: ({ color, size, focused }) => (
                        <Image
                        source={require("../asset/mypage_icon.png")}
                        style={[styles.icon, { tintColor: focused ? colorchange : 'black'}]}/>
                    ),
                    headerShown: false,
                }}/>

                {/* 노드페이지 */}
                <Tab.Screen name="Node" component={NodePage}
                options={{
                    tabBarIcon: ({ color, size, focused }) => (
                        <Image
                        source={require("../asset/nodepage_icon.png")}
                        style={[styles.icon, { tintColor: focused ? colorchange : 'black'}]}/>
                    ),
                    headerShown: false,
                }}/>

                {/* 메인페이지 */}
                <Tab.Screen
                name="Main"
                component={MainPage}
                options={{
                    tabBarIcon: ({ color, size, focused }) => (
                        <Image
                        source={require("../asset/mainpage_icon.png")}
                        style={[styles.icon, { tintColor: focused ? colorchange : 'black'}]}/>
                    ),
                    headerShown: false,
                }}
                />

                {/* 알림페이지 */}
                <Tab.Screen name="Alert" component={AlertPage}
                options={{
                    tabBarIcon: ({ color, size, focused }) => (
                        <Image
                        source={require("../asset/alertpage_icon.png")}
                        style={[styles.icon, { tintColor: focused ? colorchange : 'black'}]}/>
                    ),
                    headerShown: false,
                }}/>

                {/* 설정페이지 */}
                <Tab.Screen name="Setting" component={SettingPage}
                options={{
                    tabBarIcon: ({ color, size, focused }) => (
                        <Image
                        source={require("../asset/settingpage_icon.png")}
                        style={[styles.icon, { tintColor: focused ? colorchange : 'black'}]}/>
                    ),
                    headerShown: false,
                }}/>
            </Tab.Navigator>
    );
};

const styles = StyleSheet.create({
    container: {
        backgroundColor: 'white',
    },

    icon: {
        width: 35,
        height: 35,
    },
});

export default BottomTabNavigation;
