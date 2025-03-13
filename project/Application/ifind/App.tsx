import AsyncStorage from '@react-native-async-storage/async-storage';
import { NavigationContainer } from '@react-navigation/native';
import { createNativeStackNavigator } from '@react-navigation/native-stack';
import { useEffect, useState } from 'react';
import { enableLatestRenderer } from 'react-native-maps';
import { SafeAreaProvider } from 'react-native-safe-area-context';
import BottomTabNavigation from './src/navigation/BottomTabNavigation';
import { LoginPage } from './src/screens/LoginPage';
import { SignupPage } from './src/screens/SignupPage';

enableLatestRenderer();

const Stack = createNativeStackNavigator();

export default function App() {

  const [isUserLoggedIn, setIsUserLoggedIn] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    const checkLoginStatus = async () => {
      try {
        const uid = await AsyncStorage.getItem('userUID');
        setIsUserLoggedIn(uid !== null);
        setIsLoading(false);
      } catch (e) {
        console.error(e);
        setIsLoading(false);
      }
    };

    checkLoginStatus();
  }, []);

  if (isLoading) {
    return null;
  }

  return (
    <SafeAreaProvider>
      <NavigationContainer>
      <Stack.Navigator
        screenOptions={{ headerShown: false }}
        initialRouteName={isUserLoggedIn ? "MainPage" : "LoginPage"}
      >
        {/* 로그인 여부와 상관없이 모든 스크린을 렌더링합니다. */}
        <Stack.Screen name="MainPage" component={BottomTabNavigation} />
        <Stack.Screen name="LoginPage" component={LoginPage} />
        <Stack.Screen name="SignupPage" component={SignupPage} />
      </Stack.Navigator>
      </NavigationContainer>

    </SafeAreaProvider>
  );
}
